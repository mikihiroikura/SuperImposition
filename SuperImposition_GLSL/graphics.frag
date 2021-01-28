#version 460
//
// graphics.frag
// gl_FragColor‚Íbuilt in•Ï”
//.vert‚Ìout‚Æ“¯‚¶•Ï”
in vec2 vertextexCoords;

uniform sampler2D texture;

void main(void)
{
  vec4 color = texture2D(texture, vertextexCoords);
  gl_FragColor = color;
}

/*in vec4 vertexColor;
 
void main(void)
{
  gl_FragColor = vertexColor;
}*/