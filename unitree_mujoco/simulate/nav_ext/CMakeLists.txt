# unitree_mujoco_nav: stock simulator + simulated range scanner over DDS.
# Stand-alone project so the original simulate/CMakeLists.txt stays untouched.
#
# Build (from unitree_mujoco/simulate):
#   cmake -S nav_ext -B build_nav && cmake --build build_nav -j4
# The binary MUST live in simulate/build_nav/ so that it finds
# simulate/config.yaml and ../unitree_robots exactly like the stock binary.
cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco_nav)

set(SIM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

file(GLOB SIM_SRC
  ${SIM_DIR}/src/joystick/joystick.cc
  ${SIM_DIR}/mujoco/simulate/glfw_*.cc
  ${SIM_DIR}/mujoco/simulate/platform_*.cc
  ${SIM_DIR}/mujoco/simulate/simulate.cc
  ${SIM_DIR}/src/lodepng/lodepng.cpp
)

add_executable(unitree_mujoco_nav ${SIM_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/main_nav.cc)
target_include_directories(unitree_mujoco_nav PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${SIM_DIR}/src
  ${SIM_DIR}/mujoco/include
  ${SIM_DIR}/mujoco/simulate
  ${SIM_DIR}/src/lodepng
  /usr/local/include/iceoryx/v2.0.2
)
target_link_directories(unitree_mujoco_nav PRIVATE ${SIM_DIR}/mujoco/lib)
target_link_libraries(unitree_mujoco_nav pthread mujoco glfw yaml-cpp unitree_sdk2 boost_program_options fmt)
set_target_properties(unitree_mujoco_nav PROPERTIES BUILD_RPATH ${SIM_DIR}/mujoco/lib)
