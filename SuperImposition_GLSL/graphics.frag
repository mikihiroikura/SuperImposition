#version 460
//
// graphics.frag
// gl_FragColorはbuilt in変数
//.vertのoutと同じ変数
in vec2 texCoords;

uniform sampler2D texture;

void main(void)
{
  gl_FragColor = texture2D(texture, texCoords);
  //gl_FragColor = vec4(1.0,1.0,1.0,1.0);
}