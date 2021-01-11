#include "graphics.h"
#include <opencv2/opencv.hpp>

// constants
const unsigned int window_width = 1280;
const unsigned int window_height = 720;
GLFWwindow* window;

//shader object
static GLuint vertShader, fragShader, gl2Program;

//vbo
GLuint vbo, cbo, vao;

//imgui
GLint matlocation;

//プロトタイプ宣言
static int readShaderSource(GLuint shader, const char* file);

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

    //VAOのバインド
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    //頂点バッファオブジェクト


    //テクスチャオブジェクト


    //テクスチャ座標オブジェクト

    //VAOのアンバインド
    glBindVertexArray(0);//VAOに上のVBOの処理をまとめる，ループで一度これを呼べばvertexattrib,enablevertexattribは実行される

    //imguiの初期化
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
}

void drawGL() {

}

void finishGL() {
    //vao,vboの消去
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &cbo);
}