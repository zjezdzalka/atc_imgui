cmake_minimum_required(VERSION 3.15)
project(imgui_radar LANGUAGES CXX)

# C++ 17 :3
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GLFW Library
add_subdirectory(external/glfw)

# ImGui Library
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)

# Connect header files
add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
        external/glfw/include
)

# Run Application
add_executable(imgui_radar
        main.cpp
        function/aircraft.cpp
        function/emergency.cpp
        function/utils.cpp
        function/runway.cpp
        function/waypoint.cpp
        function/draw.cpp
        function/ils_system.h
        function/ils_system.cpp
)

target_link_libraries(imgui_radar imgui glfw)

# Make app work on windows + Linux (Wayland)
if (WIN32)
    target_link_libraries(imgui_radar opengl32)
elseif (UNIX AND NOT APPLE)
    find_package(OpenGL REQUIRED)
    target_link_libraries(imgui_radar OpenGL::GL X11 pthread dl)
endif()
