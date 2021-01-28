#version 460
//
// graphics.frag
// gl_FragColorはbuilt in変数
//.vertのoutと同じ変数
in vec2 vertextexCoords;

uniform sampler2D texture;

void main(void)
{
  vec4 color = texture2D(texture, vertextexCoords);
  //gl_FragColor = vec4(1.0,1.0,1.0,1.0);
  gl_FragColor = color
}

/*in vec4 vertexColor;
 
void main(void)
{
  gl_FragColor = vertexColor;
}*/