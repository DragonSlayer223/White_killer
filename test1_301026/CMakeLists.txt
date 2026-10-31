# Stewart platform ball tracker (C++ / OpenCV)
#
# Windows (Visual Studio + OpenCV, e.g. from opencv.org or vcpkg):
#   cmake -S . -B build -DOpenCV_DIR="C:/opencv/build"
#   cmake --build build --config Release
#   build\Release\stewart_tracker.exe --list-cameras
#
# Linux:
#   sudo apt install libopencv-dev cmake g++
#   cmake -S . -B build && cmake --build build -j

cmake_minimum_required(VERSION 3.16)
project(stewart_tracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)
find_package(Threads REQUIRED)

add_executable(stewart_tracker stewart_tracker.cpp)
target_include_directories(stewart_tracker PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stewart_tracker PRIVATE ${OpenCV_LIBS} Threads::Threads)

if(WIN32)
  target_link_libraries(stewart_tracker PRIVATE ws2_32 ole32 oleaut32 strmiids)
endif()
if(MSVC)
  target_compile_options(stewart_tracker PRIVATE /W3 /utf-8)
else()
  target_compile_options(stewart_tracker PRIVATE -Wall -Wextra)
endif()

# Copy OpenCV DLLs next to the exe on Windows (so it runs by double-click).
if(WIN32 AND OpenCV_DIR)
  file(GLOB _ocv_dlls "${OpenCV_DIR}/x64/vc*/bin/opencv_world*.dll")
  foreach(_dll ${_ocv_dlls})
    add_custom_command(TARGET stewart_tracker POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_dll}" $<TARGET_FILE_DIR:stewart_tracker>)
  endforeach()
endif()
