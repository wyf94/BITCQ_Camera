cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 3.0 REQUIRED)

FILE (GLOB ALL_SOURCES 
    "./*.cpp"
    )

add_library(GMSL_Camera ${ALL_SOURCES})

target_link_libraries(GMSL_Camera
  ${OpenCV_LIBRARIES}
)