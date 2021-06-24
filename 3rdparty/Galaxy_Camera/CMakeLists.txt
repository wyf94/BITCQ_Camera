cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 3.0 REQUIRED)

FILE (GLOB ALL_SOURCES 
    "./*.cpp"
    )

if (CMAKE_HOST_WIN32)
    set(WINDOWS 1)
elseif(CMAKE_HOST_APPLE)
    set(MACOS 1)
elseif(CMAKE_HOST_UNIX)
    set(LINUX 1)
endif()

if (WINDOWS)
    MESSAGE ( STATUS "Wrong Target System!")
elseif (LINUX)
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL x86)
        MESSAGE( STATUS "Galaxy Library Version: x86" )
        set(GalaxyLib ${PROJECT_SOURCE_DIR}/lib/Galaxy_Camera/x86/libgxiapi.so)
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL x86_64)
        MESSAGE( STATUS "Galaxy Library Version: x86_64" )
        set(GalaxyLib ${PROJECT_SOURCE_DIR}/lib/Galaxy_Camera/x86_64/libgxiapi.so)
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL armv7hf OR ARM32_BUILD)
        MESSAGE( STATUS "Galaxy Library Version: armv7hf" )
        set(GalaxyLib ${PROJECT_SOURCE_DIR}/lib/Galaxy_Camera/armv7l/libgxiapi.so)
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL aarch64) 
        MESSAGE( STATUS "Galaxy Library Version: armv8" )
        set(GalaxyLib ${PROJECT_SOURCE_DIR}/lib/Galaxy_Camera/armv8/libgxiapi.so)
    endif()
elseif (MACOS)
    MESSAGE ( STATUS "Wrong Target System!")
endif()

add_library(Galaxy_Camera ${ALL_SOURCES})

target_link_libraries(Galaxy_Camera
  ${OpenCV_LIBRARIES}
  ${GalaxyLib}
)