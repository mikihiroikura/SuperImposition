#version 460
//
// graphics.frag
// gl_FragColor��built in�ϐ�
//.vert��out�Ɠ����ϐ�
uniform sampler2D texture;

in vec2 verTexCoords;

void main(void)
{
  vec4 smpColor = texture2D(texture, verTexCoords);
  gl_FragColor = smpColor
}