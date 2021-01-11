#version 460
//
// graphics.frag
// gl_FragColor‚Íbuilt in•Ï”
//.vert‚Ìout‚Æ“¯‚¶•Ï”
in vec4 vertexColor;
 
void main(void)
{
  gl_FragColor = vertexColor;
}