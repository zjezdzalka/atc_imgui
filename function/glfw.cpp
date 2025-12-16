//
// Created by rytui on 12/3/25.
//

#include <iomanip>

// glfw error
static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}