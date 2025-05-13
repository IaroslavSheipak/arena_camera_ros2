cmake_minimum_required(VERSION 3.15)

project(arena_camera_node LANGUAGES C CXX)

# -----------------------------------------------------------------------------
# Build type and language standards
# -----------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Cross-platform optimisation flags (GCC / Clang / MSVC)
# -----------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Avoid duplicate ')' syntax errors by keeping one well-formed list
  add_compile_options(
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-ffast-math>
    $<$<CONFIG:Release>:-funsafe-math-optimizations>
    $<$<AND:$<CONFIG:Release>,$<NOT:$<STREQUAL:${CMAKE_SYSTEM_PROCESSOR},aarch64>>>:-march=native>
    $<$<AND:$<CONFIG:Release>,$<NOT:$<STREQUAL:${CMAKE_SYSTEM_PROCESSOR},aarch64>>>:-mtune=native>
    $<$<CONFIG:Release>:-funroll-loops>
    $<$<CONFIG:Release>:-fdata-sections>
    $<$<CONFIG:Release>:-ffunction-sections>
    $<$<CONFIG:Release>:-pipe>
    $<$<CONFIG:Release>:-flto>
  )
  add_link_options(
    $<$<CONFIG:Release>:-fuse-linker-plugin>
    $<$<CONFIG:Release>:-flto>
    $<$<CONFIG:Release>:-Wl,--gc-sections>
  )
elseif(MSVC)
  add_compile_options($<$<CONFIG:Release>:/O2> $<$<CONFIG:Release>:/GL>)
  add_link_options($<$<CONFIG:Release>:/LTCG> $<$<CONFIG:Release>:/OPT:REF> $<$<CONFIG:Release>:/OPT:ICF>)
endif()

# Accept ArenaSDK / GenICam headers regardless of compiler brand
add_compile_definitions(GENICAM_USER_ACCEPTS_ANY_COMPILER)

# Enable IPO / LTO where supported
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)

# -----------------------------------------------------------------------------
# Dependency discovery
# -----------------------------------------------------------------------------
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arena_sdk REQUIRED)

# Optional – link OpenMP if the tool-chain provides it (helps on aarch64)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found – building with -fopenmp support")
endif()

# -----------------------------------------------------------------------------
# Jetson Orin / Xavier NX (aarch64, Cortex-A78AE) specific tuning
# -----------------------------------------------------------------------------
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  message(STATUS "Applying Cortex-A78 optimisation flags (-mcpu=cortex-a78)")
  add_compile_options(-mcpu=cortex-a78)
  add_link_options(-mcpu=cortex-a78)
  if(OpenMP_CXX_FOUND)
    add_compile_options(-fopenmp)
    add_link_options(-fopenmp)
  endif()
endif()

# -----------------------------------------------------------------------------
# Executable: arena_camera_node / start
# -----------------------------------------------------------------------------
set(target_name start)
add_executable(${target_name}
  src/main.cpp
  src/ArenaCameraNode.cpp
  src/light_arena/deviceinfo_helper.cpp
  src/rclcpp_adapter/pixelformat_translation.cpp
  src/rclcpp_adapter/quilty_of_service_translation.cpp
)

ament_target_dependencies(
  ${target_name}
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  arena_sdk
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${target_name} OpenMP::OpenMP_CXX)
endif()

set_property(TARGET ${target_name} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

install(TARGETS ${target_name}
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Executable: trigger_image client
# -----------------------------------------------------------------------------
set(trigger_image trigger_image)
add_executable(${trigger_image} src/trigger_image_client.cpp)
ament_target_dependencies(${trigger_image} rclcpp std_srvs)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${trigger_image} OpenMP::OpenMP_CXX)
endif()
set_property(TARGET ${trigger_image} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
install(TARGETS ${trigger_image}
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Launch files and miscellaneous resources
# -----------------------------------------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()