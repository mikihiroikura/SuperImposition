#version 460
//
// graphics.frag
// gl_FragColor��built in�ϐ�
//.vert��out�Ɠ����ϐ�
in vec4 vertexColor;
 
void main(void)
{
  gl_FragColor = vertexColor;
}