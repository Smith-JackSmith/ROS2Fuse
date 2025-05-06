# CMakeLists.txt
# Copyright (c) 2025 Jack Sidman Smith
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

cmake_minimum_required(VERSION 3.8)
project(topic_fs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FUSE REQUIRED fuse)

add_executable(topic_fs src/topic_fs.cpp)

target_include_directories(topic_fs PRIVATE ${FUSE_INCLUDE_DIRS} )

ament_target_dependencies(topic_fs rclcpp std_msgs ) 

target_link_libraries(topic_fs 
                        ${FUSE_LIBRARIES} 
                         nlohmann_json::nlohmann_json
                         rclcpp::rclcpp
                         ${std_msgs_LIBRARIES} )

target_compile_options(topic_fs PRIVATE ${FUSE_CFLAGS_OTHER})                         

install(TARGETS topic_fs 
        DESTINATION lib/${PROJECT_NAME} )

install(FILES package.xml
        DESTINATION share/${PROJECT_NAME} )        

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
