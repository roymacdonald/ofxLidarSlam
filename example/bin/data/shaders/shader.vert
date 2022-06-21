#version 150
          
in vec4 position;
uniform mat4 modelViewProjectionMatrix;

uniform float range_max;
uniform float colorMapSize;
uniform vec3 offset;

out float vcolor;
void main(){
    float range = length(position.xyz - offset);
    vcolor = (range * colorMapSize)/range_max;
    gl_Position = modelViewProjectionMatrix * position;
}

