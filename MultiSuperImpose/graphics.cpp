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

//�v���g�^�C�v�錾
static int readShaderSource(GLuint shader, const char* file);

/*
** �V�F�[�_�[�̃\�[�X�v���O�������������ɓǂݍ���
*/
static int readShaderSource(GLuint shader, const char* file)
{
    FILE* fp;
    const GLchar* source;
    GLsizei length;
    int ret;

    /* �t�@�C�����J�� */
    fp = fopen(file, "rb");
    if (fp == NULL) {
        perror(file);
        return -1;
    }

    /* �t�@�C���̖����Ɉړ������݈ʒu (�܂�t�@�C���T�C�Y) �𓾂� */
    fseek(fp, 0L, SEEK_END);
    length = ftell(fp);

    /* �t�@�C���T�C�Y�̃��������m�� */
    source = (GLchar*)malloc(length);
    if (source == NULL) {
        fprintf(stderr, "Could not allocate read buffer.\n");
        return -1;
    }

    /* �t�@�C����擪����ǂݍ��� */
    fseek(fp, 0L, SEEK_SET);
    ret = fread((void*)source, 1, length, fp) != (size_t)length;
    fclose(fp);

    /* �V�F�[�_�̃\�[�X�v���O�����̃V�F�[�_�I�u�W�F�N�g�ւ̓ǂݍ��� */
    if (ret)
        fprintf(stderr, "Could not read file: %s.\n", file);
    else
        glShaderSource(shader, 1, &source, &length);

    /* �m�ۂ����������̊J�� */
    free((void*)source);

    return ret;
}

//OpenGL�̏�����
void initGL() {
    //GLFW�̏�����
    if (glfwInit() == GL_FALSE)
    {
        std::cerr << "Can't initilize GLFW" << std::endl;
    }

    //Window�̍쐬
    window = glfwCreateWindow(window_width, window_height, "SuperImposition", NULL, NULL);
    if (window == nullptr)
    {
        std::cerr << "Can't create GLFW window." << std::endl;
        glfwTerminate();
    }

    //Window��OpenGL�̑Ώۂɂ���
    glfwMakeContextCurrent(window);
    //MakeCOntextcurrent�̌�ɍs��Ȃ��Ǝ��s����炵��
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Can't initilize GLEW" << std::endl;
    }

    // default initialization
    glClearColor(0.0, 0.0, 0.0, 1.0);   //�w�i�F�̎w��
    glDisable(GL_DEPTH_TEST);

    //shader�I�u�W�F�N�g�̍쐬
    vertShader = glCreateShader(GL_VERTEX_SHADER);
    fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    if (readShaderSource(vertShader, "graphics.vert")) exit(1);
    if (readShaderSource(fragShader, "graphics.frag")) exit(1);

    //Shader compile
    glCompileShader(vertShader);
    glCompileShader(fragShader);

    //�v���O�����I�u�W�F�N�g�쐬
    gl2Program = glCreateProgram();
    glAttachShader(gl2Program, vertShader);
    glDeleteShader(vertShader);
    glAttachShader(gl2Program, fragShader);
    glDeleteShader(fragShader);

    //�V�F�[�_�v���O�����̃����N
    glLinkProgram(gl2Program);

    //�V�F�[�_�v���O�����̃����N
    glLinkProgram(gl2Program);

    matlocation = glGetUniformLocation(gl2Program, "MVP");//�V�F�[�_�v���O�������"MVP" uniform�̈ʒu�̌���

    //VAO�̃o�C���h
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    //���_�o�b�t�@�I�u�W�F�N�g


    //�e�N�X�`���I�u�W�F�N�g


    //�e�N�X�`�����W�I�u�W�F�N�g

    //VAO�̃A���o�C���h
    glBindVertexArray(0);//VAO�ɏ��VBO�̏������܂Ƃ߂�C���[�v�ň�x������Ăׂ�vertexattrib,enablevertexattrib�͎��s�����

    //imgui�̏�����
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
}

void drawGL() {

}

void finishGL() {
    //vao,vbo�̏���
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &cbo);
}