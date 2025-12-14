cmake_minimum_required(VERSION 3.15)
project(imgui_radar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# --- GLFW ---
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG latest
)
FetchContent_MakeAvailable(glfw)

# --- ImGui ---
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG docking
)
FetchContent_MakeAvailable(imgui)

# Utwórz bibliotekę z plików ImGui (z backendami GLFW + OpenGL3)
add_library(imgui_build
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
        function/aircraft.h
        function/emergency.cpp
        function/emergency.h
        function/emergency_types.h
        function/utils.cpp
        function/utils.h
        function/aircraft.cpp
)

# Dodaj ścieżki do nagłówków ImGui i GLFW (to właśnie naprawia Twój błąd!)
target_include_directories(imgui_build PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${glfw_SOURCE_DIR}/include
)

# Twój plik źródłowy
add_executable(imgui_radar main.cpp
        function/aircraft.h
        function/aircraft.cpp
        function/emergency.cpp
        function/emergency.h
        function/utils.cpp
        function/utils.h
        function/emergency_types.h
        function/aircraft.cpp)

# Linkowanie
target_link_libraries(imgui_radar imgui_build glfw)

# Platform-specific libraries
if (WIN32)
    target_link_libraries(imgui_radar
            opengl32
            user32
            gdi32
            shell32
    )
elseif (UNIX AND NOT APPLE)  # Linux
    find_package(OpenGL REQUIRED)
    target_link_libraries(imgui_radar
            OpenGL::GL
            X11
            pthread
            dl
            rt
    )
endif()