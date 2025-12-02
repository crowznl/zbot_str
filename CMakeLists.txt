cmake_minimum_required(VERSION 3.5)
project(can_test)



include_directories(
    include
    can
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3")

link_libraries(pthread)
link_directories (${PROJECT_SOURCE_DIR}/can)


# CAN收发节点
option(USE_LIBTORCH "Enable linking with LibTorch (TorchScript)" ON)
# Default libtorch path (used if user didn't provide CMAKE_PREFIX_PATH)
# You can override by passing -DCMAKE_PREFIX_PATH=/your/libtorch/path to cmake
if (USE_LIBTORCH)
    if (NOT DEFINED CMAKE_PREFIX_PATH OR CMAKE_PREFIX_PATH STREQUAL "")
        set(CMAKE_PREFIX_PATH "/home/rain/libtorch" CACHE PATH "Default path to LibTorch")
        message(STATUS "CMAKE_PREFIX_PATH not provided; defaulting to /home/rain/libtorch")
    else()
        message(STATUS "CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")
    endif()
endif()
# Example: 
# cmake -DUSE_LIBTORCH=OFF -DCMAKE_PREFIX_PATH=/home/rain/libtorch ..
# make -j2

add_executable(can_node src/Tangair_usb2can.cpp src/can_node.cpp)
target_link_libraries(can_node
    pthread
    usb_can
    ${CMAKE_THREAD_LIBS_INIT}
    ${CMAKE_DL_LIBS}
)

if (USE_LIBTORCH)
    find_package(Torch REQUIRED)
    if (Torch_FOUND)
        target_compile_definitions(can_node PRIVATE USE_LIBTORCH=1)
        target_link_libraries(can_node "${TORCH_LIBRARIES}")
        message(STATUS "Linking with LibTorch: ${TORCH_LIBRARIES}")
    else()
        message(WARNING "LibTorch requested but not found. Build will continue without Torch.")
    endif()
endif()





