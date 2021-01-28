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

//�o�͓_�Q�Ɋւ���p�����[�^
float realsense_pc[vert_cnt * realsense_cnt][3] = { 0 };



//�v���g�^�C�v�錾
static void setfov(GLFWwindow* window, double x, double y);
static int readShaderSource(GLuint shader, const char* file);


static void setfov(GLFWwindow* window, double x, double y) {
    fov -= static_cast<GLfloat>(y);
}
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
    texturelocation = glGetUniformLocation(gl2Program, "texture");//�V�F�[�_�v���O�������"texture" uniform�̈ʒu�̌���

    //VAO�̃o�C���h
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    //���_�o�b�t�@�I�u�W�F�N�g
    glGenBuffers(1, &vbo);//vbp�쐬
    glBindBuffer(GL_ARRAY_BUFFER, vbo);//vbo�̃o�C���h�C���ꂩ��̏����̑Ώ�
    glBufferData(GL_ARRAY_BUFFER, sizeof(realsense_pc), nullptr, GL_DYNAMIC_DRAW);//vbo�̃f�[�^�̈�̊m��
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);//vertex shader���̈����̎w��index�ɍ����悤�ɕύX����
    glEnableVertexAttribArray(0);//index�̒l��attribute�ϐ��̗L����
    //glEnableVertexArrayAttrib(vao, 0); //��̊֐��̑���ɂ���ł�����


    //�e�N�X�`���I�u�W�F�N�g


    //�e�N�X�`�����W�I�u�W�F�N�g
    glGenBuffers(1, &tcbo);
    glBindBuffer(GL_ARRAY_BUFFER, tcbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vert_cnt * 2 * realsense_cnt, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    //glEnableVertexArrayAttrib(vao, 1);

    //VAO�̃A���o�C���h
    glBindBuffer(GL_ARRAY_BUFFER, 0); //EnableVertexAttribArray�̌�ɍs��
    glBindVertexArray(0);//VAO�ɏ��VBO�̏������܂Ƃ߂�C���[�v�ň�x������Ăׂ�vertexattrib,enablevertexattrib�͎��s�����

    //�X�N���[������Callback����֐��̎w��
    glfwSetScrollCallback(window, setfov);

    //imgui�̏�����
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
}

void drawGL_realsense(float *pts0, int *pc0_ringid, float *pc0_texcoords) {
    //�_�Q�̈ʒu�X�V
    memcpy(&realsense_pc[0][0], pts0 + (unsigned long long)*pc0_ringid * 3 * vert_cnt, sizeof(float) * 3 * vert_cnt);

    //�V�F�[�_�v���O�����̊J�n
    glUseProgram(gl2Program);
    //glUniformMatrix4fv(matlocation, 1, GL_FALSE, &mvp[0][0]); //�V�F�[�_�v���O�����̊J�n�̌�ɃV�F�[�_�v���O��������MVP�s����X�V

    //�_�Q�̈ʒu�ƐF���X�V
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(realsense_pc), realsense_pc);
    glBindBuffer(GL_ARRAY_BUFFER, tcbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(pc0_texcoords), pc0_texcoords);//VBO���̃e�N�X�`�����W���X�V
    glBindVertexArray(vao);//VBO�ł̓_�Q�ʒu�ƃe�N�X�`�����W�X�V���܂Ƃ߂�VAO���o�C���h���Ď��s
    glDrawArrays(GL_POINTS, 0, vert_cnt * 3);//���ۂ̕`��
    glBindVertexArray(0);//VBO�̃A���o�C���h

    glfwPollEvents(); //�}�E�X�C�x���g�����o���L�^

    //start imgui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSize(ImVec2(320, 300), ImGuiCond_Once);
    ImGui::Begin("Logs and Parameters");
    //ImGui::Text("Processing time %.3f ms", gltime * 1000);
    //ImGui::Text("DDMotor mode  :  %c", mode);
    //ImGui::Text("Rotation speed:  %d", rpm);
    //hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem); //IMGUI���Window�ł̃J�[�\���������̃t���O�𗧂Ă�
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


    glfwSwapBuffers(window);//�_�u���o�b�t�@�̓���ւ��C������s�����Ƃŉ�ʂ��ꕔ�X�V�������̂ňꕔ�X�V����Ă��Ȃ��Ȃǂ̂��������Ȃ�����

}

void finishGL() {
    //vao,vbo�̏���
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &tcbo);
}