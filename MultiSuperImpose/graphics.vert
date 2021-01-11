#version 460 core
//
// graphics.vert
//�@gl_Position��built in�ϐ�
//�w��Index�̕ϐ�
layout (location = 0) in vec3 position; // the position variable has attribute position 0
layout (location = 1) in vec3 color;

//shader program����Global�ϐ�
uniform mat4 MVP;
//out�̕ϐ���.frag��in�����O����v���Ă���Έ����p�����
out vec4 vertexColor;

void main()
{
    vec4 v = vec4(position, 1.0);
    gl_Position = MVP * v;
    vertexColor = vec4(color, 1.0);
}