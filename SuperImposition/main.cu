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
#include "realsense_glfw.h"

#define MAX_EPSILON_ERROR 10.0f
#define THRESHOLD          0.30f
#define REFRESH_DELAY     10 //ms

using namespace std;

////////////////////////////////////////////////////////////////////////////////
// constants
const unsigned int window_width = 1280;
const unsigned int window_height = 720;

const unsigned int mesh_width = 106*8;
const unsigned int mesh_height = 60*8;

glm::mat4 mvp;
GLuint Matrix;

//shader object
GLFWwindow* window;
static GLuint vertShader;
static GLuint fragShader;
GLuint gl2Program;

GLuint vao;

// vbo variables
GLuint vbo, tcbo;
struct cudaGraphicsResource* cuda_vbo_resource, *cuda_tcbo_resource;
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
const rs2::vertex* vertices;
const rs2::texture_coordinate* tex_coords;

////////////////////////////////////////////////////////////////////////////////
// declaration, forward
bool runTest(int argc, char** argv, char* ref_file);

// GL functionality
bool initGL(int* argc, char** argv);
void deleteVBO(GLuint* vbo, struct cudaGraphicsResource* vbo_res);

// Cuda functionality
void runCuda(struct cudaGraphicsResource** vbo_resource, const rs2::vertex* vertex);

const char* sSDKsample = "simpleGL (VBO)";

///////////////////////////////////////////////////////////////////////////////
//! Simple kernel to modify vertex positions in sine wave pattern
//! @param data  data in global memory
///////////////////////////////////////////////////////////////////////////////
__global__ void simple_vbo_kernel(float3* pos, const rs2::vertex* vertex, unsigned int width, unsigned int height, float rot_x, float rot_y, float trans_z)
{
    //処理する位置
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    //座標変換
    


    // write output vertex
    //pos[y * width + x] = make_float3((vertex+ y * width + x)->x, (vertex + y * width + x)->y, (vertex + y * width + x)->z);
    pos[y * width + x] = make_float3(x*0.01, y*0.01, 0);
}


void launch_kernel(float3* pos, const rs2::vertex* vertex, float rot_x, float rot_y, float trans_z)
{
    // execute the kernel
    dim3 block(8, 8, 1);
    dim3 grid(mesh_width / block.x, mesh_height / block.y, 1);
    simple_vbo_kernel << < grid, block >> > (pos, vertex, mesh_width, mesh_height, rot_x, rot_y, trans_z);
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
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    unsigned int size_vert = 407040 * 3 * sizeof(float);
    glBufferData(GL_ARRAY_BUFFER, size_vert, vertices, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    checkCudaErrors(cudaGraphicsGLRegisterBuffer(&cuda_vbo_resource, vbo, cudaGraphicsRegisterFlagsWriteDiscard));//CUDAのグラフィックスリソースに登録する

    //Tex coordinateバッファオブジェクト
    glGenBuffers(1, &tcbo);
    glBindBuffer(GL_ARRAY_BUFFER, tcbo);
    unsigned int size_uv = 407040 * 2 * sizeof(float);
    glBufferData(GL_ARRAY_BUFFER, size_uv, tex_coords, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    checkCudaErrors(cudaGraphicsGLRegisterBuffer(&cuda_tcbo_resource, tcbo, cudaGraphicsRegisterFlagsWriteDiscard));//CUDAのグラフィックスリソースに登録する

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
        vertices = points.get_vertices();
        runCuda(&cuda_vbo_resource, vertices);
        //draw_pointcloud(vertices, &vbo, tex_coords, &tcbo, window_width, window_height, tex, points, translate_z, rotate_x, rotate_y);
        draw_pointcloud2(&vbo, tex_coords, &tcbo, window_width, window_height, tex, points, translate_z, rotate_x, rotate_y);

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
void runCuda(struct cudaGraphicsResource** vbo_resource, const rs2::vertex* vertex)
{
    // map OpenGL buffer object for writing from CUDA
    float3* dptr;
    checkCudaErrors(cudaGraphicsMapResources(1, vbo_resource, 0));
    size_t num_bytes;
    checkCudaErrors(cudaGraphicsResourceGetMappedPointer((void**)&dptr, &num_bytes,
        *vbo_resource));
    //printf("CUDA mapped VBO: May access %ld bytes\n", num_bytes);

    // execute the kernel
    //    dim3 block(8, 8, 1);
    //    dim3 grid(mesh_width / block.x, mesh_height / block.y, 1);
    //    kernel<<< grid, block>>>(dptr, mesh_width, mesh_height, g_fAnim);

    launch_kernel(dptr, vertex, rotate_x, rotate_y, translate_z);

    // unmap buffer object
    checkCudaErrors(cudaGraphicsUnmapResources(1, vbo_resource, 0));
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