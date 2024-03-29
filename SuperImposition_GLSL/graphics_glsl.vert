#version 460 core
//
// graphics.vert
//　gl_Positionはbuilt in変数
//指定Indexの変数
layout (location = 0) in vec3 position; // the position variable has attribute position 0
layout (location = 1) in vec2 texcoord;

//shader program内のGlobal変数
uniform mat4 MVP;
//outの変数と.fragのinが名前が一致していれば引き継がれる
out vec2 verTexCoords;

void main()
{
    vec4 v = vec4(position, 1.0);
    gl_Position = MVP * v;
    verTexCoords = texcoord;
}