#version 460
//
// graphics.frag
// gl_FragColorはbuilt in変数
//.vertのoutと同じ変数
in vec4 vertexColor;
 
void main(void)
{
  gl_FragColor = vertexColor;
}