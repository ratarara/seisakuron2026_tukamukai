cmake_minimum_required(VERSION 3.8)
project(tukamukai)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ソルバー（ROS に依存しないライブラリ）
add_library(tukamukai_solver src/solver.cpp)
target_include_directories(tukamukai_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 実行ファイル。ノードを作ったらここに add_executable を追加し、install(TARGETS) にも足す
add_executable(solver_demo src/solver_demo.cpp)
target_link_libraries(solver_demo tukamukai_solver)

install(TARGETS tukamukai_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS solver_demo
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # 著作権表示と cpplint の著作権チェックは、LICENSE が決まるまでスキップする
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_solver test/test_solver.cpp)
  target_link_libraries(test_solver tukamukai_solver)
endif()

ament_package()
