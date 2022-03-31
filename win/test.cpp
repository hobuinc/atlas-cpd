#include <iostream>

#include <GLFW/glfw3.h>

#include "linmath.h"

static const struct
{
    float x, y;
    float r, g, b;
} vertices[3] = 
{
    { -0.6f, -0.4f, 1.f, 0.f, 0.f },
    {  0.6f, -0.4f, 0.f, 1.f, 0.f },
    {   0.f,  0.6f, 0.f, 0.f, 1.f }
};

static const char* vertexShaderText =
"#version 110\n"
"uniform mat4 MVP;\n"
"attribute vec3 vCol;\n"
"attribute vec2 vPos;\n"
"varying vec3 color;\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(vPos, 0.0, 1.0);\n"
"    color = vCol;\n"
"}\n";
static const char* fragmentShaderText =
"#version 110\n"
"varying vec3 color;\n"
"void main()\n"
"{\n"
"    gl_FragColor = vec4(color, 1.0);\n"
"}\n";

class Foo
{
public:
    bool init();
    void run();
    void terminate();

    GLFWwindow *m_window;
    GLuint m_program;
    GLint m_mvpLocation;
};

void Foo::run()
{
    while (!glfwWindowShouldClose(m_window))
    {
        int width, height;
        float ratio;
        mat4x4 m, p, mvp;

        glfwGetFramebufferSize(m_window, &width, &height);
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);

        mat4x4_identity(m);
        mat4x4_rotate_Z(m, m, (float)glfwGetTime());
        mat4x4_ortho(p, -ratio, ratio, -1, 1, 1, -1);
        mat4x4_mul(mvp, p, m);

        glUseProgram(m_program);
        glUniformMatrix4fv(m_mvpLocation, 1, GL_FALSE, (const GLfloat *)mvp);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        glfwSwapBuffers(m_window);
        glfwPollEvents();
    }
}

int main()
{
    Foo f;
    f.init();
    f.run();
    f.terminate();
    return 0;
    /**
    GLFWwindow *window = init();
    if (window)
    {
        Foo f(window);
        f.run();
        terminate(window);
    }
    **/
    return 0;
}

bool Foo::init()
{
    auto errorCb = [](int error, const char *description)
    {
        std::cerr << "GL error (" << error << ")  " << description << "\n";
    };

    auto keyCb = [](GLFWwindow *window, int key, int scancode, int action,
        int mode)
    {
        std::cerr << "Key pressed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GLFW_TRUE);
    };

    glfwSetErrorCallback(errorCb);
    if (!glfwInit())
        return false;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    m_window = glfwCreateWindow(640, 480, "Title", NULL, NULL);
    if (!m_window)
        return false;

    glfwSetKeyCallback(m_window, keyCb);
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    GLuint vertexBuffer;
    glGenBuffers(1, &vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderText, NULL);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderText, NULL);
    glCompileShader(fragmentShader);

    m_program = glCreateProgram();
    glAttachShader(m_program, vertexShader);
    glAttachShader(m_program, fragmentShader);
    glLinkProgram(m_program);

    m_mvpLocation = glGetUniformLocation(m_program, "MVP");
    GLuint vposLocation = glGetUniformLocation(m_program, "vPos");
    GLuint vcolLocation = glGetUniformLocation(m_program, "vCol");

    glEnableVertexAttribArray(vposLocation);
    glVertexAttribPointer(vposLocation, 2, GL_FLOAT, GL_FALSE,
        sizeof(vertices[0]), nullptr);
    glEnableVertexAttribArray(vcolLocation);
    glVertexAttribPointer(vcolLocation, 3, GL_FLOAT, GL_FALSE,
        sizeof(vertices[0]), (void *)(sizeof(float) * 2));
    return true;
}

void Foo::terminate()
{
    if (m_window)
        glfwDestroyWindow(m_window);
    glfwTerminate();
}

