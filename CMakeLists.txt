cmake_minimum_required(VERSION 3.8)
project(rossealfs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
)

include_directories(
  include/rossealfs
)

add_executable(rossealfs
  src/main.cpp
  src/RosSealfs.cpp
  src/LogTopic.cpp
  src/Decoder.cpp
  src/StringDecoder.cpp
  src/RosoutDecoder.cpp
  src/DecoderFactory.cpp
)

ament_target_dependencies(rossealfs ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
install(TARGETS
  rossealfs
  DESTINATION lib/${PROJECT_NAME})

ament_package()
