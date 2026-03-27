// Vertex shader: pass world-space Z to fragment shader
varying float height;

void main()
{
    vec4 world_pos = gl_ModelViewMatrix * gl_Vertex;
    height = gl_Vertex.z;  // Z in model space = elevation
    gl_Position = ftransform();
    gl_FrontColor = gl_Color;
}
