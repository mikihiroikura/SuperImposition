#version 460
//
// graphics.frag
// gl_FragColorはbuilt in変数
//.vertのoutと同じ変数
in vec4 vertexColor;
in vec2 texCoords;

uniform sampler2D texture;
 
void main(void)
{
  //gl_FragColor = vertexColor * texture2D(texture, texCoords);
  gl_FragColor = texture2D(texture, texCoords);
}