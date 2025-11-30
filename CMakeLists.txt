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
)

# Dodaj ścieżki do nagłówków ImGui i GLFW (to właśnie naprawia Twój błąd!)
target_include_directories(imgui_build PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${glfw_SOURCE_DIR}/include
)

# Twój plik źródłowy
add_executable(imgui_radar main.cpp)

# Linkowanie
target_link_libraries(imgui_radar PRIVATE
    imgui_build
    glfw
    OpenGL32.lib
    user32.lib
    gdi32.lib
    shell32.lib
)
