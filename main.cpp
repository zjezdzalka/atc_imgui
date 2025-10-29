#define GLFW_INCLUDE_NONE

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>

int main() {
    if (!glfwInit()) {
        std::cout << "Error in initalizing glfw\n";
        return -1;
    }

    GLFWwindow *window =
        glfwCreateWindow(400, 400, "OpenGL Tutorial", NULL, NULL);

    if (!window) {
        std::cout << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    if (!gladLoadGL()) {
        std::cout << "Failed to initalize glad\n";
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    std::cout << "Opengl Version\t" << GLVersion.major << "." << GLVersion.minor
              << std::endl;

    while (!glfwWindowShouldClose(window)) {
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}