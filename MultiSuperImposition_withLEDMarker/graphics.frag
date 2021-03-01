#version 460
//
// graphics.frag
// gl_FragColor��built in�ϐ�
//.vert��out�Ɠ����ϐ�
in vec2 texCoords;

uniform sampler2D texture;

void main(void)
{
  gl_FragColor = texture2D(texture, texCoords) * vec4(1.0,1.0,1.0,0.5);
}