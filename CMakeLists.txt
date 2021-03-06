project(MPC)
option(ENABLE_VISUALIZATION "enable visualization, requires openc" OFF)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp)

include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})
target_link_libraries(mpc ipopt z ssl uv uWS)

if (ENABLE_VISUALIZATION)
    add_definitions(-DENABLE_VISUALIZATION)
    target_sources(mpc PUBLIC src/Color.cpp src/ViewPort.cpp src/Drawing.cpp)
    target_link_libraries(mpc opencv_core opencv_highgui opencv_imgproc)
endif(ENABLE_VISUALIZATION)

