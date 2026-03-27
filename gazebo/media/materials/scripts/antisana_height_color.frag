// Fragment shader: height-based terrain coloring
// Green lowlands → brown highlands → white snow peaks
varying float height;

uniform float green_max;  // top of green band
uniform float brown_max;  // top of brown band
uniform float snow_min;   // start of snow

void main()
{
    vec3 green = vec3(0.30, 0.50, 0.20);  // páramo grassland
    vec3 brown = vec3(0.45, 0.35, 0.20);  // rocky highlands
    vec3 grey  = vec3(0.55, 0.52, 0.50);  // scree/rock
    vec3 white = vec3(0.95, 0.96, 0.98);  // glacial snow

    vec3 color;
    if (height < green_max) {
        color = green;
    } else if (height < brown_max) {
        float t = (height - green_max) / (brown_max - green_max);
        color = mix(green, brown, t);
    } else if (height < snow_min) {
        float t = (height - brown_max) / (snow_min - brown_max);
        color = mix(brown, grey, t);
    } else {
        float t = clamp((height - snow_min) / 200.0, 0.0, 1.0);
        color = mix(grey, white, t);
    }

    gl_FragColor = vec4(color, 1.0);
}
