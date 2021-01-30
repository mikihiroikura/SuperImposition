#version 460
//
// graphics.frag
// gl_FragColor‚Íbuilt in•Ï”
//.vert‚Ìout‚Æ“¯‚¶•Ï”
in vec4 vertexColor;
in vec2 texCoords;

uniform sampler2D texture;
 
void main(void)
{
  gl_FragColor = vertexColor * texture2D(texture, texCoords);
}