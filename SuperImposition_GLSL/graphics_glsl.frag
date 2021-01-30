#version 460
//
// graphics.frag
// gl_FragColor‚Íbuilt in•Ï”
//.vert‚Ìout‚Æ“¯‚¶•Ï”
uniform sampler2D texture;

in vec2 verTexCoords;

void main(void)
{
  vec4 smpColor = texture2D(texture, verTexCoords);
  gl_FragColor = smpColor
}