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

//imgui
float init_fov = 60, fov = init_fov;

//出力点群に関するパラメータ
float realsense_pc[vert_cnt * realsense_cnt][3] = { 0 };



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
    glBufferData(GL_ARRAY_BUFFER, sizeof(realsense_pc), nullptr, GL_DYNAMIC_DRAW);//vboのデータ領域の確保
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);//vertex shader内の引数の指定indexに合うように変更する
    glEnableVertexAttribArray(0);//indexの値のattribute変数の有効化
    //glEnableVertexArrayAttrib(vao, 0); //上の関数の代わりにこれでもいい


    //テクスチャオブジェクト


    //テクスチャ座標オブジェクト
    glGenBuffers(1, &tcbo);
    glBindBuffer(GL_ARRAY_BUFFER, tcbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vert_cnt * 2 * realsense_cnt, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    //glEnableVertexArrayAttrib(vao, 1);

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

void drawGL_realsense(float *pts0, int *pc0_ringid, float *pc0_texcoords) {
    //点群の位置更新
    memcpy(&realsense_pc[0][0], pts0 + (unsigned long long)*pc0_ringid * 3 * vert_cnt, sizeof(float) * 3 * vert_cnt);

    //シェーダプログラムの開始
    glUseProgram(gl2Program);
    //glUniformMatrix4fv(matlocation, 1, GL_FALSE, &mvp[0][0]); //シェーダプログラムの開始の後にシェーダプログラム内のMVP行列を更新

    //点群の位置と色を更新
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(realsense_pc), realsense_pc);
    glBindBuffer(GL_ARRAY_BUFFER, tcbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(pc0_texcoords), pc0_texcoords);//VBO内のテクスチャ座標を更新
    glBindVertexArray(vao);//VBOでの点群位置とテクスチャ座標更新をまとめたVAOをバインドして実行
    glDrawArrays(GL_POINTS, 0, vert_cnt * 3);//実際の描画
    glBindVertexArray(0);//VBOのアンバインド

    glfwPollEvents(); //マウスイベントを取り出し記録

    //start imgui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSize(ImVec2(320, 300), ImGuiCond_Once);
    ImGui::Begin("Logs and Parameters");
    //ImGui::Text("Processing time %.3f ms", gltime * 1000);
    //ImGui::Text("DDMotor mode  :  %c", mode);
    //ImGui::Text("Rotation speed:  %d", rpm);
    //hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem); //IMGUI上のWindowでのカーソル処理時のフラグを立てる
    //ImGui::Checkbox("Hide Red", &hide_red);
    //ImGui::Checkbox("Hide Green", &hide_green);
    //ImGui::Checkbox("Hide Blue", &hide_blue);
    //ImGui::DragFloat("rotate x", &rotate_x);
    //ImGui::DragFloat("rotate y", &rotate_y);
    //ImGui::DragFloat("trans x", &translate_x);
    //ImGui::DragFloat("trans y", &translate_y);
    //ImGui::DragFloat("trans z", &translate_z);
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