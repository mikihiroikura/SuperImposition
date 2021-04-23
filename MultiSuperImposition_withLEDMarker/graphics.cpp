#include <cmath>
#define _USE_MATH_DEFINES

#include "graphics.h"
#include <opencv2/opencv.hpp>

#pragma warning(disable:4996)

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
float rotate_x[realsense_cnt] = { 0.0 }, rotate_y[realsense_cnt] = { 0.0 }, rotate_z[realsense_cnt] = { 0.0 };
float translate_x[realsense_cnt] = { 0.0 }, translate_y[realsense_cnt] = { 0.0 }, translate_z[realsense_cnt] = { -.0 };
float camtarget[3] = { 0,0,0 };
double mouse_x, mouse_y, mouse_x_old, mouse_y_old;
double horiz_angle = -M_PI, vert_angle = 0.0;
int trans_max = 10;
double mouse_speed = 0.01;
double dx = 0.0, dy = 0.0;
float hovered;
bool rs[2] = { true, true };
bool userelpose = true;
glm::vec3 position(0, 0, -1), up(0, -1, 0), direction(0, 0, 0);
glm::mat4 mvp, vp, Model[2], View, Projection;
float campos_radius = 1.0;

//プロトタイプ宣言
static void setfov(GLFWwindow* window, double x, double y);
static int readShaderSource(GLuint shader, const char* file);


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
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
    glBufferData(GL_ARRAY_BUFFER, vert_cnt * 3 * sizeof(float) * realsense_cnt, nullptr, GL_DYNAMIC_DRAW);//vboのデータ領域の確保
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);//vertex shader内の引数の指定indexに合うように変更する
    glEnableVertexAttribArray(0);//indexの値のattribute変数の有効化
    //glEnableVertexArrayAttrib(vao, 0); //上の関数の
    

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
    cv::Mat dummy = cv::Mat(colorheight, colorwidth, CV_8UC3, cv::Scalar::all(255));
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, colorwidth, colorheight, 0, GL_RGB, GL_UNSIGNED_BYTE, (void*)dummy.data);
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

void drawGL_realsense(float** pts, float** texcoords, rs2::frame** colorframes, glm::mat4* rtuavrs2ugvrs) {
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
        for (size_t i = 0; i < realsense_cnt; i++)
        {
            rotate_x[i] = 0.0, rotate_y[i] = 0.0, rotate_z[i] = 0.0;
            translate_x[i] = 0.0, translate_y[i] = 0.0, translate_z[i] = -.0;
            rs[i] = true;
        }
        fov = init_fov;
        camtarget[0] = 0, camtarget[1] = 0, camtarget[2] = 0;
        campos_radius = 1.0;
    }

    //Model view行列の計算
    position = glm::vec3(campos_radius * cos(vert_angle) * sin(horiz_angle), campos_radius * sin(vert_angle), campos_radius * cos(vert_angle) * cos(horiz_angle));
    Projection = glm::perspective(glm::radians(fov), (GLfloat)window_width / (GLfloat)window_height, 0.1f, 100.0f);
    direction.x = camtarget[0]; direction.y = camtarget[1]; direction.z = camtarget[2];
    View = glm::lookAt(position, direction, up);
    for (size_t i = 0; i < realsense_cnt; i++)
    {
        Model[i] = glm::translate(glm::mat4(1.0), glm::vec3(translate_x[i], translate_y[i], translate_z[i]))
            * glm::rotate(glm::radians(rotate_x[i]), glm::vec3(1, 0, 0))
            * glm::rotate(glm::radians(rotate_y[i]), glm::vec3(0, 1, 0))
            * glm::rotate(glm::radians(rotate_z[i]), glm::vec3(0, 0, 1));
        if (i == 1 && userelpose) Model[i] *= *rtuavrs2ugvrs;
    }
    vp = Projection * View;

    //シェーダプログラムの開始
    glUseProgram(gl2Program);

    //点群の位置とテクスチャ座標を更新
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    for (size_t i = 0; i < realsense_cnt; i++)
    {
        glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * vert_cnt * 3 * i, sizeof(float) * vert_cnt * 3, pts[i]);
    }
    glBindBuffer(GL_ARRAY_BUFFER, tcbo);
    for (size_t i = 0; i < realsense_cnt; i++)
    {
        glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * vert_cnt * 2 * i, sizeof(float) * vert_cnt * 2, texcoords[i]);//VBO内のテクスチャ座標を更新
    }
    glBindVertexArray(vao);//VBOでの点群位置とテクスチャ座標更新をまとめたVAOをバインドして実行
    glBindTexture(GL_TEXTURE_2D, tex);
    for (size_t i = 0; i < realsense_cnt; i++)
    {
        if (rs[i])
        {
            mvp = vp * Model[i];
            glUniformMatrix4fv(matlocation, 1, GL_FALSE, &mvp[0][0]); //シェーダプログラムの開始の後にシェーダプログラム内のMVP行列を更新
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, colorwidth, colorheight, GL_RGB, GL_UNSIGNED_BYTE, (void*)colorframes[i]->get_data());
            glDrawArrays(GL_POINTS, vert_cnt * i, vert_cnt);//実際の描画
        }
    }
    glBindVertexArray(0);//VBOのアンバインド

    glfwPollEvents(); //マウスイベントを取り出し記録

    //start imgui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSize(ImVec2(320, 350), ImGuiCond_Once);
    ImGui::Begin("Logs and Parameters");
    hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem); //IMGUI上のWindowでのカーソル処理時のフラグを立てる
    ImGui::Checkbox("Use RelPose", &userelpose);
    ImGui::Checkbox("RealSense 0", &rs[0]);
    ImGui::SameLine();
    ImGui::Checkbox("RealSense 1", &rs[1]);
    ImGui::SliderFloat2("rotate_x", (float*)&rotate_x, -180.0f, 180.0f, "%.0f");
    ImGui::SliderFloat2("rotate_y", (float*)&rotate_y, -180.0f, 180.0f, "%.0f");
    ImGui::SliderFloat2("rotate_z", (float*)&rotate_z, -180.0f, 180.0f, "%.0f");
    ImGui::InputInt("translate_max", &trans_max);
    ImGui::SliderFloat2("translate_x", (float*)&translate_x, -trans_max, trans_max, "%.0f");
    ImGui::SliderFloat2("translate_y", (float*)&translate_y, -trans_max, trans_max, "%.0f");
    ImGui::SliderFloat2("translate_z", (float*)&translate_z, -trans_max, trans_max, "%.0f");
    ImGui::Text("Camera Position & View target");
    ImGui::DragFloat("campos radius", &campos_radius);
    ImGui::DragFloat("viewtarget x", &camtarget[0]);
    ImGui::DragFloat("viewtarget y", &camtarget[1]);
    ImGui::DragFloat("viewtarget z", &camtarget[2]);
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    //ダブルバッファの入れ替え，これを行うことで画面が一部更新したもので一部更新されていないなどのがたつきをなくせる
    glfwSwapBuffers(window);
    
}

void finishGL() {
    //vao,vboの消去
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &tcbo);
    glDeleteTextures(1, &tex);
}