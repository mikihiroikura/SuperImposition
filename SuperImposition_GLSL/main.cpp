// includes, system
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#define _USE_MATH_DEFINES

#include <algorithm>            // std::min, std::max
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Windows.h>

#include <librealsense2/rs.hpp>
#include "realsense.h"
#include <thread>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtx/transform.hpp>

// imgui include
#include "imgui/imgui.h"	
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"

const int vert_cnt = 921600;
const int realsense_cnt = 1;

rs2::context context;
const int ring_size_realsense = 5;
int getpc_id = 0;
float* texcoords_src;
int update_ringid;

//時間に関する変数
LARGE_INTEGER start, stop, freq;

#pragma warning(disable:4996)
struct PointCloud
{
	const rs2::vertex* pc_buffer;
	const rs2::texture_coordinate* texcoords;
	float pc_ringbuffer[ring_size_realsense * vert_cnt][3];
	float texcoords_ringbuffer[ring_size_realsense * vert_cnt][2];
    rs2::frame colorframe_buffer[ring_size_realsense];
	int pc_ringid = 0;
};

PointCloud pc0;

//OpenGL
// constants
const unsigned int window_width = 1280;
const unsigned int window_height = 720;
GLFWwindow* window;

//shader object
static GLuint vertShader, fragShader, gl2Program;

//vbo
GLuint vbo, tcbo, vao;
GLint matlocation;
GLint texturelocation;
GLuint tex;

//imgui
float init_fov = 60, fov = init_fov;
float pointsize = 2.5;
bool hide_red, hide_green, hide_blue;
float rotate_x = 0.0, rotate_y = 0.0;
float translate_x = 0.0, translate_y = 0.0, translate_z = -.0;
double mouse_x, mouse_y, mouse_x_old, mouse_y_old;
double horiz_angle = -M_PI, vert_angle = 0.0;
double mouse_speed = 0.01;
double dx = 0.0, dy = 0.0;
float hovered;
glm::vec3 position(0, 0, -1), up(0, -1, 0), direction(0, 0, 0);
glm::mat4 mvp, Model, View, Projection;

//出力点群に関するパラメータ
float realsense_pc[vert_cnt * realsense_cnt][3] = { 0 };
float realsense_texcoord[vert_cnt * realsense_cnt][2] = { 0 };

//プロトタイプ宣言
void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc);
static void setfov(GLFWwindow* window, double x, double y);
static int readShaderSource(GLuint shader, const char* file);
void initGL();
void finishGL();
void drawGL_realsense(float* pts0, int* pc0_ringid, float* pc0_texcoords, rs2::frame* colorframes);


int main() {
	//パラメータ
	bool flg = true;
    if (!QueryPerformanceFrequency(&freq)) { return 0; }// 単位習得

	//ログ初期化
	cout << "Set PointCloud buffers....";
	
	//pc0.pc_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 3);
	//pc0.texcoords_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 2);
	memset(pc0.pc_ringbuffer, 0, sizeof(float) * ring_size_realsense * vert_cnt * 3);
	memset(pc0.texcoords_ringbuffer, 0, sizeof(float) * ring_size_realsense * vert_cnt * 2);

	pc0.pc_buffer = (rs2::vertex*)malloc(sizeof(float) * vert_cnt * 3);
	pc0.texcoords = (rs2::texture_coordinate*)malloc(sizeof(float) * vert_cnt * 2);
	cout << "OK!" << endl;

	//Realsenseの初期化
	cout << "Set RealsenseD435..........";
	const rs2::device_list device_list = context.query_devices();
	rs2::device device = device_list[0];
	realsense rs(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_RGB8, RS2_FORMAT_Z16);
	cout << "OK!" << endl;


	//OpenGLの初期化
	initGL();

	//スレッドの作成
	thread thr1(GetPointClouds, &rs, &flg, &pc0);

	//メインループ
	cout << "Main loop start!" << endl;
    QueryPerformanceCounter(&start);
    double timer = 0;
    while (timer < 1.5)
    {
        QueryPerformanceCounter(&stop);
        timer = (double)(stop.QuadPart - start.QuadPart) / freq.QuadPart;
    }
	while (flg)
	{
		if (GetAsyncKeyState(VK_SPACE) & 0x8000) flg = false;
		getpc_id = pc0.pc_ringid;
        if (getpc_id < 0) getpc_id + ring_size_realsense;
		texcoords_src = &pc0.texcoords_ringbuffer[0][0];
		drawGL_realsense(&pc0.pc_ringbuffer[0][0], &getpc_id, texcoords_src, &pc0.colorframe_buffer[0]);
	}

	//OpenGLの終了
	finishGL();


	//スレッド削除
	if (thr1.joinable())thr1.join();
	//if (thr2.joinable())thr2.join();

	//動的メモリの開放
	//free(pc0.pc_ringbuffer);
	//free(pc0.pc_buffer);
	//free(pc1.pc_buffer);

	return 0;
}


void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc) {
	for (size_t i = 0; i < 30; i++) { rs->update_frame(); }
	while (*flg)
	{
		//点群計算
		rs->update_frame();
		rs->update_color();
		rs->update_depth();
		rs->calc_pointcloud();
		pc->pc_buffer = rs->points.get_vertices();
        //cout << "pointssize " << rs->points.size() << endl;
		pc->texcoords = rs->points.get_texture_coordinates();
		//取得点群をリングバッファに保存
		memcpy((&pc->pc_ringbuffer[0][0] + (unsigned long long)pc->pc_ringid * vert_cnt * 3), pc->pc_buffer, sizeof(float) * vert_cnt * 3);
		memcpy((&pc->texcoords_ringbuffer[0][0] + (unsigned long long)pc->pc_ringid * vert_cnt * 2), pc->texcoords, sizeof(float) * vert_cnt * 2);
        pc->colorframe_buffer[pc->pc_ringid] = rs->colorframe;

		//リングバッファの番号を更新
		pc->pc_ringid = (pc->pc_ringid + 1) % ring_size_realsense;
	}
}


static void setfov(GLFWwindow* window, double x, double y) {
    fov -= static_cast<GLfloat>(y);
}
/*
** シェーダーのソースプログラムをメモリに読み込む
*/
static int readShaderSource(GLuint shader, const char* file)
{
    FILE* fp;
    const GLchar* source;
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
    source = (GLchar*)malloc(length);
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

//OpenGLの初期化
void initGL() {
    //GLFWの初期化
    if (glfwInit() == GL_FALSE)
    {
        std::cerr << "Can't initilize GLFW" << std::endl;
    }

    //Windowの作成
    window = glfwCreateWindow(window_width, window_height, "SuperImposition", NULL, NULL);
    if (window == nullptr)
    {
        std::cerr << "Can't create GLFW window." << std::endl;
        glfwTerminate();
    }

    //WindowをOpenGLの対象にする
    glfwMakeContextCurrent(window);
    //MakeCOntextcurrentの後に行わないと失敗するらしい
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Can't initilize GLEW" << std::endl;
    }

    // default initialization
    glClearColor(0.0, 0.0, 0.0, 1.0);   //背景色の指定
    glDisable(GL_DEPTH_TEST);

    //shaderオブジェクトの作成
    vertShader = glCreateShader(GL_VERTEX_SHADER);
    fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    if (readShaderSource(vertShader, "graphics.vert")) exit(1);
    if (readShaderSource(fragShader, "graphics.frag")) exit(1);

    //Shader compile
    glCompileShader(vertShader);
    glCompileShader(fragShader);

    //プログラムオブジェクト作成
    gl2Program = glCreateProgram();
    glAttachShader(gl2Program, vertShader);
    glDeleteShader(vertShader);
    glAttachShader(gl2Program, fragShader);
    glDeleteShader(fragShader);

    //シェーダプログラムのリンク
    glLinkProgram(gl2Program);

    matlocation = glGetUniformLocation(gl2Program, "MVP");//シェーダプログラム上の"MVP" uniformの位置の検索
    texturelocation = glGetUniformLocation(gl2Program, "texture");//シェーダプログラム上の"texture" uniformの位置の検索

    

    //VAOのバインド
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    //頂点バッファオブジェクト
    glGenBuffers(1, &vbo);//vbp作成
    glBindBuffer(GL_ARRAY_BUFFER, vbo);//vboのバインド，これからの処理の対象
    glBufferData(GL_ARRAY_BUFFER, vert_cnt * 3 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);//vboのデータ領域の確保
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);//vertex shader内の引数の指定indexに合うように変更する
    glEnableVertexAttribArray(0);//indexの値のattribute変数の有効化
    //glEnableVertexArrayAttrib(vao, 0); //上の関数の代わりにこれでもいい

    //テクスチャ座標オブジェクト
    glGenBuffers(1, &tcbo);
    glBindBuffer(GL_ARRAY_BUFFER, tcbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vert_cnt * 2 * realsense_cnt, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    //glEnableVertexArrayAttrib(vao, 1);

    //テクスチャオブジェクト
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    cv::Mat dummy = cv::Mat(1080, 1920, CV_8UC3, cv::Scalar::all(255));
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1920, 1080, 0, GL_RGB, GL_UNSIGNED_BYTE, (void*)dummy.data);
    float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    //VAOのアンバインド
    glBindBuffer(GL_ARRAY_BUFFER, 0); //EnableVertexAttribArrayの後に行う
    glBindVertexArray(0);//VAOに上のVBOの処理をまとめる，ループで一度これを呼べばvertexattrib,enablevertexattribは実行される

    //スクロール時にCallbackする関数の指定
    glfwSetScrollCallback(window, setfov);

    //imguiの初期化
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
}

void drawGL_realsense(float* pts0, int* pc0_ringid, float* pc0_texcoords, rs2::frame* colorframes) {
    //点群の位置更新
    memcpy(&realsense_pc[0][0], pts0 + (unsigned long long) * pc0_ringid * 3 * vert_cnt, sizeof(float) * 3 * vert_cnt);
    memcpy(&realsense_texcoord[0][0], pc0_texcoords + (unsigned long long) * pc0_ringid * 2 * vert_cnt, sizeof(float) * 2 * vert_cnt);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    //カーソル位置から移動変化量を計算
    glfwGetCursorPos(window, &mouse_x, &mouse_y);
    dx = mouse_x - mouse_x_old;
    dy = mouse_y - mouse_y_old;

    //左クリックしていればかつIMGUI上のWindowにいなければ，移動変化量を基に角度更新
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && !hovered)
    {
        horiz_angle += mouse_speed * dx;
        vert_angle += mouse_speed * dy;
    }
    mouse_x_old = mouse_x;
    mouse_y_old = mouse_y;

    //スペースキーを押していれば，パラメータリセット
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
    {
        horiz_angle = -M_PI;
        vert_angle = 0.0;
        rotate_x = 0.0, rotate_y = 0.0;
        translate_x = 0.0, translate_y = 0.0, translate_z = -.0;
        fov = init_fov;
        hide_red = false;
        hide_green = false;
        hide_blue = false;
    }

    //Model view行列の計算
    position = glm::vec3(cos(vert_angle) * sin(horiz_angle), sin(vert_angle), cos(vert_angle) * cos(horiz_angle));
    Projection = glm::perspective(glm::radians(fov), (GLfloat)window_width / (GLfloat)window_height, 0.1f, 100.0f);
    View = glm::lookAt(position, direction, up);
    Model = glm::translate(glm::mat4(1.0), glm::vec3(translate_x, translate_y, translate_z))
        * glm::rotate(glm::radians(rotate_x), glm::vec3(1, 0, 0))
        * glm::rotate(glm::radians(rotate_y), glm::vec3(0, 1, 0));
    mvp = Projection * View * Model;

    //シェーダプログラムの開始
    glUseProgram(gl2Program);
    glUniformMatrix4fv(matlocation, 1, GL_FALSE, &mvp[0][0]); //シェーダプログラムの開始の後にシェーダプログラム内のMVP行列を更新

    //テクスチャの更新
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1920, 1080, GL_RGB, GL_UNSIGNED_BYTE, (void*)colorframes[*pc0_ringid].get_data());
    
    //点群の位置とテクスチャ座標を更新
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(realsense_pc), realsense_pc);
    glBindBuffer(GL_ARRAY_BUFFER, tcbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(realsense_texcoord), realsense_texcoord);//VBO内のテクスチャ座標を更新
    glTexCoordPointer(2, GL_FLOAT, 0, 0);
    glBindVertexArray(vao);//VBOでの点群位置とテクスチャ座標更新をまとめたVAOをバインドして実行
    glDrawArrays(GL_POINTS, 0, vert_cnt);//実際の描画

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);//VBOのアンバインド

    glfwPollEvents(); //マウスイベントを取り出し記録

    //start imgui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSize(ImVec2(320, 300), ImGuiCond_Once);
    ImGui::Begin("Logs and Parameters");
    hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem); //IMGUI上のWindowでのカーソル処理時のフラグを立てる
    ImGui::DragFloat("rotate x", &rotate_x);
    ImGui::DragFloat("rotate y", &rotate_y);
    ImGui::DragFloat("trans x", &translate_x);
    ImGui::DragFloat("trans y", &translate_y);
    ImGui::DragFloat("trans z", &translate_z);
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


    glfwSwapBuffers(window);//ダブルバッファの入れ替え，これを行うことで画面が一部更新したもので一部更新されていないなどのがたつきをなくせる

}

void finishGL() {
    //vao,vboの消去
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &tcbo);
}