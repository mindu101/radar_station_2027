cmake_minimum_required(VERSION 3.10)
project(radar_fusion CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

add_library(fusion STATIC
    src/config.cpp
    src/fusion.cpp)
target_include_directories(fusion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(fusion_demo tools/fusion_demo.cpp)
target_link_libraries(fusion_demo PRIVATE fusion)

add_executable(fusion_udp_listener tools/udp_listener.cpp)
target_link_libraries(fusion_udp_listener PRIVATE fusion)

add_executable(fusion_tests tests/test_fusion.cpp)
target_link_libraries(fusion_tests PRIVATE fusion)

# 相机程序（上一届 radar_station）直接链接的适配器。没有 OpenCV 时跳过。
find_package(Threads REQUIRED)
find_package(OpenCV QUIET COMPONENTS core)
if(OpenCV_FOUND)
    add_library(fusion_camera STATIC src/camera_fusion.cpp)
    target_include_directories(fusion_camera PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(fusion_camera PUBLIC fusion Threads::Threads ${OpenCV_LIBS})

    add_executable(camera_fusion_tests tests/test_camera_fusion.cpp)
    target_link_libraries(camera_fusion_tests PRIVATE fusion_camera)
    add_test(NAME camera_fusion_tests COMMAND camera_fusion_tests)
    message(STATUS "fusion_camera enabled (OpenCV ${OpenCV_VERSION})")
else()
    message(STATUS "OpenCV not found: fusion_camera disabled")
endif()

add_test(NAME fusion_tests COMMAND fusion_tests)
