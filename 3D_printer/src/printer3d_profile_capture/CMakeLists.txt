cmake_minimum_required(VERSION 3.10)
project(printer3d_profile_capture)

# Standards C/C++
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dépendances (doit venir avant add_executable)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(printer3d_gocator_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Boost REQUIRED COMPONENTS system)

# Chemins GO_SDK (avant add_executable)
set(GO_SDK_4 ${CMAKE_CURRENT_SOURCE_DIR}/external/GO_SDK)
find_path(GOCATOR_INCLUDES NAMES GoSdk/GoSdk.h PATHS ${GO_SDK_4}/Gocator/GoSdk)
find_path(KAPI_INCLUDES NAMES kApi/kApi.h PATHS ${GO_SDK_4}/Platform/kApi)
find_library(GOCATOR_LIBRARIES NAMES GoSdk PATHS ${GO_SDK_4}/lib/linux_x64d)
find_library(KAPI_LIBRARIES NAMES kApi PATHS ${GO_SDK_4}/lib/linux_x64d)

# Sources
set(LIB_SRCS src/driver/gocatorSensor.cpp)
set(LIB_HDRS src/driver/gocatorSensor.h)

# --- Création de la cible ---
add_executable(gocator_sensor_node
    src/nodes/gocatorSensorNode.cpp
    ${LIB_SRCS}
)

# --- Configuration de la cible (APRES add_executable) ---
target_include_directories(gocator_sensor_node PRIVATE
    ${GOCATOR_INCLUDES}
    ${KAPI_INCLUDES}
    ${PCL_INCLUDE_DIRS}
)

target_link_directories(gocator_sensor_node PRIVATE
    ${PCL_LIBRARY_DIRS}
    ${GO_SDK_4}/lib/linux_x64d
)

target_link_libraries(gocator_sensor_node
    ${GOCATOR_LIBRARIES}
    ${KAPI_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
)

ament_target_dependencies(gocator_sensor_node
    rclcpp
    sensor_msgs
    std_msgs
    printer3d_gocator_msgs
    pcl_conversions
    Boost
)

# Installation
install(TARGETS gocator_sensor_node DESTINATION lib/${PROJECT_NAME})
install(
    DIRECTORY external/GO_SDK/lib/linux_x64d/
    DESTINATION lib/${PROJECT_NAME}/
)

# Tests
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()