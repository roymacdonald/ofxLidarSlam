#version 150
          
in vec4 position;
uniform mat4 modelViewProjectionMatrix;

uniform float range_max;
uniform float colorMapSize;
          
out float vcolor;
void main(){
    float range = length(position.xyz);
    vcolor = 1000*(range * colorMapSize)/(22632.f*range_max);
    gl_Position = modelViewProjectionMatrix * position;
}

