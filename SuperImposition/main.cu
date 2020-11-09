////////////////////////////////////////////////////////////////////////////
//
// Copyright 1993-2015 NVIDIA Corporation.  All rights reserved.
//
// Please refer to the NVIDIA end user license agreement (EULA) associated
// with this source code for terms and conditions that govern your use of
// this software. Any use, reproduction, disclosure, or distribution of
// this software and related documentation outside the terms of the EULA
// is strictly prohibited.
//
////////////////////////////////////////////////////////////////////////////

/*
    This example demonstrates how to use the Cuda OpenGL bindings to
    dynamically modify a vertex buffer using a Cuda kernel.

    The steps are:
    1. Create an empty vertex buffer object (VBO)
    2. Register the VBO with Cuda
    3. Map the VBO for writing from Cuda
    4. Run Cuda kernel to modify the vertex positions
    5. Unmap the VBO
    6. Render the results using OpenGL

    Host code
*/

// includes, system
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <algorithm>            // std::min, std::max
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Windows.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtx/transform.hpp>

#ifdef _WIN32
#  define WINDOWS_LEAN_AND_MEAN
#  define NOMINMAX
#  include <windows.h>
#endif

// includes, cuda
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

// Utilities and timing functions
#include <helper_functions.h>    // includes cuda.h and cuda_runtime_api.h
#include <timer.h>               // timing functions

// CUDA helper functions
#include <helper_cuda.h>         // helper functions for CUDA error check
#include <vector_types.h>

// imgui include
#include "imgui/imgui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"

#include <librealsense2/rs.hpp>
#include "realsense_glfw.hpp"

#define MAX_EPSILON_ERROR 10.0f
#define THRESHOLD          0.30f
#define REFRESH_DELAY     10 //ms

using namespace std;

////////////////////////////////////////////////////////////////////////////////
// constants
const unsigned int window_width = 1280;
const unsigned int window_height = 720;

const unsigned int mesh_width = 256;
const unsigned int mesh_height = 256;

glm::mat4 mvp;
GLuint Matrix;

//shader object
GLFWwindow* window;
static GLuint vertShader;
static GLuint fragShader;
GLuint gl2Program;

GLuint vao;

// vbo variables
GLuint vbo;
struct cudaGraphicsResource* cuda_vbo_resource;
void* d_vbo_buffer = NULL;

float g_fAnim = 0.0;

// mouse controls
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
float rotate_x = 0.0, rotate_y = 0.0;
float translate_z = -3.0;

StopWatchInterface* timer = NULL;

// Auto-Verification Code
int fpsCount = 0;        // FPS count for averaging
int fpsLimit = 1;        // FPS limit for sampling
int g_Index = 0;
float avgFPS = 0.0f;
unsigned int frameCount = 0;
unsigned int g_TotalErrors = 0;
bool g_bQAReadback = false;

int* pArgc = NULL;
char** pArgv = NULL;

#define MAX(a,b) ((a > b) ? a : b)

// Realsense
rs2::pointcloud pc;
rs2::points points;
rs2::pipeline pipe;
rs2::frameset frames;
texture_gl tex;

////////////////////////////////////////////////////////////////////////////////
// declaration, forward
bool runTest(int argc, char** argv, char* ref_file);

// GL functionality
bool initGL(int* argc, char** argv);
void createVBO(GLuint* vbo, struct cudaGraphicsResource** vbo_res,
    unsigned int vbo_res_flags);
void deleteVBO(GLuint* vbo, struct cudaGraphicsResource* vbo_res);

// rendering callbacks
int readShaderSource(GLuint shader, const char* file);

// Cuda functionality
void runCuda(struct cudaGraphicsResource** vbo_resource);

const char* sSDKsample = "simpleGL (VBO)";

///////////////////////////////////////////////////////////////////////////////
//! Simple kernel to modify vertex positions in sine wave pattern
//! @param data  data in global memory
///////////////////////////////////////////////////////////////////////////////
__global__ void simple_vbo_kernel(float4* pos, unsigned int width, unsigned int height, float time)
{
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    // calculate uv coordinates
    float u = x / (float)width;
    float v = y / (float)height;
    u = u * 2.0f - 1.0f;
    v = v * 2.0f - 1.0f;

    // calculate simple sine wave pattern
    float freq = 4.0f;
    float w = sinf(u * freq + time) * cosf(v * freq + time) * 0.5f;

    //座標変換
    float ud = cosf(time) * u + sinf(time) * v;
    float vd = -sinf(time) * u + cosf(time) * v;


    // write output vertex
    pos[y * width + x] = make_float4(ud, w, vd, 1.0f);
}


void launch_kernel(float4* pos, unsigned int mesh_width,
    unsigned int mesh_height, float time)
{
    // execute the kernel
    dim3 block(8, 8, 1);
    dim3 grid(mesh_width / block.x, mesh_height / block.y, 1);
    simple_vbo_kernel << < grid, block >> > (pos, mesh_width, mesh_height, time);
}

////////////////////////////////////////////////////////////////////////////////
// Program main
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    char* ref_file = NULL;

    pArgc = &argc;
    pArgv = argv;

#if defined(__linux__)
    setenv("DISPLAY", ":0", 0);
#endif

    printf("%s starting...\n", sSDKsample);

    if (argc > 1)
    {
        if (checkCmdLineFlag(argc, (const char**)argv, "file"))
        {
            // In this mode, we are running non-OpenGL and doing a compare of the VBO was generated correctly
            getCmdLineArgumentString(argc, (const char**)argv, "file", (char**)&ref_file);
        }
    }

    printf("\n");

    pipe.start();
    for (size_t i = 0; i < 30; i++)
    {
        frames = pipe.wait_for_frames();
    }

    runTest(argc, argv, ref_file);

    printf("%s completed, returned %s\n", sSDKsample, (g_TotalErrors == 0) ? "OK" : "ERROR!");
    exit(g_TotalErrors == 0 ? EXIT_SUCCESS : EXIT_FAILURE);
}

////////////////////////////////////////////////////////////////////////////////
//! Initialize GL
////////////////////////////////////////////////////////////////////////////////
bool initGL(int* argc, char** argv)
{
    //GLFWの初期化
    if (glfwInit() == GL_FALSE)
    {
        std::cerr << "Can't initilize GLFW" << std::endl;
        return 1;
    }

    //Windowの作成
    window = glfwCreateWindow(window_width, window_height, "Cuda GL Interop (VBO)", NULL, NULL);
    if (window == nullptr)
    {
        std::cerr << "Can't create GLFW window." << std::endl;
        glfwTerminate();
        return 1;
    }
    
    //WindowをOpenGLの対象にする
    glfwMakeContextCurrent(window);
    //MakeCOntextcurrentの後に行わないと失敗するらしい
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Can't initilize GLEW" << std::endl;
        return 1;
    }

    // default initialization
    glClearColor(0.0, 0.0, 0.0, 1.0);   //背景色の指定
    glDisable(GL_DEPTH_TEST);
    
    //vertShader = glCreateShader(GL_VERTEX_SHADER);
    //fragShader = glCreateShader(GL_FRAGMENT_SHADER);

    //////ソースプログラム読み込み
    //if (readShaderSource(vertShader, "points.vert")) exit(1);
    //if (readShaderSource(fragShader, "points.frag")) exit(1);

    //////Shaderコンパイル
    //glCompileShader(vertShader);
    //glCompileShader(fragShader);

    //////プログラムオブジェクトの作成
    //gl2Program = glCreateProgram();
    //glAttachShader(gl2Program, vertShader);
    //glAttachShader(gl2Program, fragShader);
    //glDeleteShader(vertShader);
    //glDeleteShader(fragShader);

    ////プログラムオブジェクトのリンク
    //glBindAttribLocation(gl2Program, 0, "position");
    //glBindFragDataLocation(gl2Program, 0, "gl_FragColor");
    //glLinkProgram(gl2Program);

    ////頂点配列オブジェクト
    //glGenVertexArrays(1, &vao);
    //glBindVertexArray(vao);

    //頂点バッファオブジェクト
    /*glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    unsigned int size = mesh_width * mesh_height * 4 * sizeof(float);
    glBufferData(GL_ARRAY_BUFFER, size, nullptr, GL_DYNAMIC_DRAW);*/

    ////Vertexshaderの参照
    //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    //glEnableVertexAttribArray(0);

    ////頂点バッファオブジェクトの結合解除
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
    //glBindVertexArray(0);


    // viewport
    glViewport(0, 0, window_width, window_height);

    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)window_width / (GLfloat)window_height, 0.1, 10.0);

    // カメラ行列
    //glm::mat4 View = glm::lookAt(
    //    glm::vec3(4, 4, 4), // ワールド空間でカメラは(4,3,3)にあります。
    //    glm::vec3(0, 0, 0), // 原点を見ています。
    //    glm::vec3(0, 1, 0)  // 頭が上方向(0,-1,0にセットすると上下逆転します。)
    //);
    //glm::mat4 Projection = glm::perspective(glm::radians(60.0f), 4.0f / 3.0f, 0.1f, 10.0f);
    //mvp = Projection;// *View;

    //Matrix = glGetUniformLocation(gl2Program, "MVP");

    //imguiの初期化
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    return true;
}

/*
** シェーダーのソースプログラムをメモリに読み込む
*/
int readShaderSource(GLuint shader, const char* file)
{
    FILE* fp;
    const char* source;
    GLsizei length;
    int ret;

    /* ファイルを開く */
    fp = fopen(file, "rb");
    if (fp == NULL) {
        perror(file);
        return -1;
    }

    /* ファイルの末尾に移動し現在位置 (つまりファイルサイズ) を得る */
    fseek(fp, 0L, SEEK_END);
    length = ftell(fp);

    /* ファイルサイズのメモリを確保 */
    source = (char*)malloc(length);
    if (source == NULL) {
        fprintf(stderr, "Could not allocate read buffer.\n");
        return -1;
    }

    /* ファイルを先頭から読み込む */
    fseek(fp, 0L, SEEK_SET);
    ret = fread((void*)source, 1, length, fp) != (size_t)length;
    fclose(fp);

    /* シェーダのソースプログラムのシェーダオブジェクトへの読み込み */
    if (ret)
        fprintf(stderr, "Could not read file: %s.\n", file);
    else
        glShaderSource(shader, 1, &source, &length);

    /* 確保したメモリの開放 */
    free((void*)source);

    return ret;
}


////////////////////////////////////////////////////////////////////////////////
//! Run a simple test for CUDA
////////////////////////////////////////////////////////////////////////////////
bool runTest(int argc, char** argv, char* ref_file)
{
    // Create the CUTIL timer
    sdkCreateTimer(&timer);

    // use command-line specified CUDA device, otherwise use device with highest Gflops/s
    int devID = findCudaDevice(argc, (const char**)argv);

    // First initialize OpenGL context, so we can properly set the GL for CUDA.
        // This is necessary in order to achieve optimal performance with OpenGL/CUDA interop.
    if (false == initGL(&argc, argv))
    {
        return false;
    }

    //// create VBO
    //createVBO(&vbo, &cuda_vbo_resource, cudaGraphicsMapFlagsWriteDiscard);

    //// run the cuda part
    //runCuda(&cuda_vbo_resource);

    //ここにループを書く
    while (glfwWindowShouldClose(window) == GL_FALSE)
    {
        // run the cuda part
        /*runCuda(&cuda_vbo_resource);*/

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0); //これでカメラの上向きの軸を--y方向にすることで上下を合わせる
        glRotated(rotate_x, 1, 0, 0);
        glRotated(rotate_y, 0, 1, 0);
        glTranslatef(0, 0, translate_z);

        //realsense
        auto frames = pipe.wait_for_frames();
        auto color = frames.get_color_frame();
        pc.map_to(color);
        auto depth = frames.get_depth_frame();
        points = pc.calculate(depth);
        tex.upload(color);
        draw_pointcloud(window_width, window_height, tex, points, translate_z, rotate_x, rotate_y);

        /*glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexPointer(4, GL_FLOAT, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glColor3f(1.0, 0.0, 0.0);
        glDrawArrays(GL_POINTS, 0, mesh_width * mesh_height);
        glDisableClientState(GL_VERTEX_ARRAY);*/
        glPopMatrix();

        glfwPollEvents();

        //start imgui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("hello world");
        ImGui::Text("This is useful text");
        ImGui::DragFloat("rotate x", &rotate_x);
        ImGui::DragFloat("rotate y", &rotate_y);
        ImGui::DragFloat("trans z", &translate_z);
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwSwapBuffers(window);

        g_fAnim += 0.01f;
    }

    deleteVBO(&vbo, cuda_vbo_resource);

    return true;
}

////////////////////////////////////////////////////////////////////////////////
//! Run the Cuda part of the computation
////////////////////////////////////////////////////////////////////////////////
void runCuda(struct cudaGraphicsResource** vbo_resource)
{
    // map OpenGL buffer object for writing from CUDA
    float4* dptr;
    checkCudaErrors(cudaGraphicsMapResources(1, vbo_resource, 0));
    size_t num_bytes;
    checkCudaErrors(cudaGraphicsResourceGetMappedPointer((void**)&dptr, &num_bytes,
        *vbo_resource));
    //printf("CUDA mapped VBO: May access %ld bytes\n", num_bytes);

    // execute the kernel
    //    dim3 block(8, 8, 1);
    //    dim3 grid(mesh_width / block.x, mesh_height / block.y, 1);
    //    kernel<<< grid, block>>>(dptr, mesh_width, mesh_height, g_fAnim);

    launch_kernel(dptr, mesh_width, mesh_height, g_fAnim);

    // unmap buffer object
    checkCudaErrors(cudaGraphicsUnmapResources(1, vbo_resource, 0));
}


////////////////////////////////////////////////////////////////////////////////
//! Create VBO
////////////////////////////////////////////////////////////////////////////////
void createVBO(GLuint* vbo, struct cudaGraphicsResource** vbo_res,
    unsigned int vbo_res_flags)
{
    assert(vbo);

    // create buffer object
    glGenBuffers(1, vbo);
    glBindBuffer(GL_ARRAY_BUFFER, *vbo);

    // initialize buffer object
    unsigned int size = mesh_width * mesh_height * 4 * sizeof(float);
    glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // register this buffer object with CUDA
    checkCudaErrors(cudaGraphicsGLRegisterBuffer(vbo_res, *vbo, vbo_res_flags));
}

////////////////////////////////////////////////////////////////////////////////
//! Delete VBO
////////////////////////////////////////////////////////////////////////////////
void deleteVBO(GLuint* vbo, struct cudaGraphicsResource* vbo_res)
{

    // unregister this buffer object with CUDA
    checkCudaErrors(cudaGraphicsUnregisterResource(vbo_res));

    glBindBuffer(1, *vbo);
    glDeleteBuffers(1, vbo);

    *vbo = 0;
}


//////////////////////////////////////////////////////////////////////////////////
////! Display callback
//////////////////////////////////////////////////////////////////////////////////
//void display()
//{
//    sdkStartTimer(&timer);
//
//    // run CUDA kernel to generate vertex positions
//    runCuda(&cuda_vbo_resource);
//
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//    // set view matrix
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//    glTranslatef(0.0, 0.0, translate_z);
//    glRotatef(rotate_x, 1.0, 0.0, 0.0);
//    glRotatef(rotate_y, 0.0, 1.0, 0.0);
//
//    // render from the vbo
//    glBindBuffer(GL_ARRAY_BUFFER, vbo);
//    glVertexPointer(4, GL_FLOAT, 0, 0);
//
//    glEnableClientState(GL_VERTEX_ARRAY);
//    glColor3f(1.0, 0.0, 0.0);
//    glDrawArrays(GL_POINTS, 0, mesh_width * mesh_height);
//    glDisableClientState(GL_VERTEX_ARRAY);
//
//    glutSwapBuffers();
//
//    g_fAnim += 0.01f;
//
//    sdkStopTimer(&timer);
//    computeFPS();
//}