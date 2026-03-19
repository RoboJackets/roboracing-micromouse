cmake_minimum_required(VERSION 3.14)
project(AnalyzerUI CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15" CACHE STRING "Minimum OS X deployment version")

include(FetchContent)

# Fetch GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.3.8
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# Fetch ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.90.4
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()

# Fetch ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
)
FetchContent_GetProperties(implot)
if(NOT implot_POPULATED)
    FetchContent_Populate(implot)
endif()

add_library(imgui_lib
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC 
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw)

add_library(implot_lib
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
)
target_include_directories(implot_lib PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot_lib PUBLIC imgui_lib)

add_executable(AnalyzerUI
    src/main.cpp
    src/CSVParser.cpp
    src/Dashboard.cpp
)

target_include_directories(AnalyzerUI PRIVATE src)
target_link_libraries(AnalyzerUI PRIVATE imgui_lib implot_lib glfw)

if(APPLE)
    target_link_libraries(AnalyzerUI PRIVATE "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
else()
    target_link_libraries(AnalyzerUI PRIVATE GL dl pthread)
endif()
