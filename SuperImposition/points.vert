#version 120
//
// simple.vert
//
invariant gl_Position;
attribute vec3 position;
uniform mat4 MVP;
 
void main(void)
{
  vec4 v = vec4(position, 1.0);
  //gl_Position = MVP * v;
}