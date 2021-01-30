#version 460 core
//
// graphics.vert
//@gl_Position‚Íbuilt in•Ï”
//w’èIndex‚Ì•Ï”
layout (location = 0) in vec3 position; // the position variable has attribute position 0
layout (location = 1) in vec3 color;
layout (location = 2) in vec2 texcoord;

//shader program“à‚ÌGlobal•Ï”
uniform mat4 MVP;
//out‚Ì•Ï”‚Æ.frag‚Ìin‚ª–¼‘O‚ªˆê’v‚µ‚Ä‚¢‚ê‚Îˆø‚«Œp‚ª‚ê‚é
out vec4 vertexColor;
out vec2 texCoords;

void main()
{
    vec4 v = vec4(position, 1.0);
    gl_Position = MVP * v;
    vertexColor = vec4(color, 1.0);
    texCoords = texcoord;
}