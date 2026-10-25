cmake_minimum_required(VERSION 3.16)
project(go2_autonomy CXX)

# Autonomous navigation stack for the Unitree Go2 (MuJoCo simulation + real robot).
# Dependencies: unitree_sdk2 (installed in /opt/unitree_robotics) only.
# Optional: MuJoCo (for the offline controller-tuning tools), see -DGO2_BUILD_TOOLS.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Threads REQUIRED)

# ---- hardware-independent core (no DDS) -------------------------------------
add_library(go2_core STATIC
  src/robot/go2_kinematics.cpp
  src/locomotion/locomotion_controller.cpp
  src/estimation/state_estimator.cpp
  src/perception/occupancy_grid.cpp
  src/planning/path_planner.cpp
)
target_include_directories(go2_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(go2_core PUBLIC Threads::Threads)

# ---- Unitree SDK2 / DDS layer + runtime + navigation -------------------------
add_library(go2_autonomy STATIC
  src/comm/go2_dds_interface.cpp
  src/app/go2_robot.cpp
  src/navigation/navigator.cpp
  src/app/go2_navigation_system.cpp
)
target_link_libraries(go2_autonomy PUBLIC go2_core unitree_sdk2)

# ---- applications -------------------------------------------------------------
add_executable(go2_nav_app apps/go2_nav_app.cpp)
target_link_libraries(go2_nav_app go2_autonomy)

add_executable(go2_acceptance_tests apps/go2_acceptance_tests.cpp)
target_link_libraries(go2_acceptance_tests go2_autonomy)

add_executable(go2_scan_probe apps/go2_scan_probe.cpp)
target_link_libraries(go2_scan_probe go2_autonomy)

# ---- offline unit test (no DDS, no simulator) ------------------------------------
add_executable(test_planner tools/test_planner.cpp)
target_link_libraries(test_planner go2_core)
enable_testing()
add_test(NAME planner COMMAND test_planner)

# ---- optional offline tools (need MuJoCo headers/libs) -------------------------
option(GO2_BUILD_TOOLS "Build MuJoCo-linked offline tuning tools" OFF)
set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.3.6" CACHE PATH "MuJoCo release directory")
if (GO2_BUILD_TOOLS)
  foreach(tool mujoco_harness test_kinematics)
    add_executable(${tool} tools/${tool}.cpp)
    target_include_directories(${tool} PRIVATE ${MUJOCO_DIR}/include)
    target_link_directories(${tool} PRIVATE ${MUJOCO_DIR}/lib)
    target_link_libraries(${tool} go2_core mujoco)
    set_target_properties(${tool} PROPERTIES BUILD_RPATH ${MUJOCO_DIR}/lib)
  endforeach()
endif()
