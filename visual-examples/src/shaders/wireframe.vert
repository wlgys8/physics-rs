in vec3 position;
in vec4 instanceTransformRow0;
in vec4 instanceTransformRow1;
in vec4 instanceTransformRow2;
uniform mat4 model;
uniform mat4 viewProjection;
uniform vec3 color;
out vec4 v_color;

void main() {
    vec4 posWorld = model * vec4(position, 1.0);
    mat4 instanceMat = transpose(mat4(instanceTransformRow0, instanceTransformRow1, instanceTransformRow2, vec4(0.0, 0.0, 0.0, 1.0)));
    posWorld = instanceMat * posWorld;
    gl_Position = viewProjection * posWorld;
    v_color = vec4(color, 1.0);
}