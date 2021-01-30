#version 460
//
// graphics.frag
// gl_FragColorはbuilt in変数
//.vertのoutと同じ変数
uniform sampler2D texture;

in vec2 verTexCoords;

void main(void)
{
  vec4 smpColor = texture2D(texture, verTexCoords);
  gl_FragColor = smpColor
}