#version 150
                    
in float vcolor;
uniform sampler2DRect palette;
out vec4 color;
void main() {
    // color = vec4(vcolor,vcolor,vcolor,1.0);
  color = texture(palette, vec2(vcolor, 1));
}
          
