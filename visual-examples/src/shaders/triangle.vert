in vec3 position;
uniform mat4 model;
uniform mat4 viewProjection;
uniform vec3 color;
out vec4 v_color;

void main() {
    gl_Position = viewProjection * model * vec4(position, 1.0);
    v_color = vec4(color, 1.0);
}