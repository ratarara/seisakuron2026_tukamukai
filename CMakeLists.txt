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
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ソルバー（ROS に依存しないライブラリ）
add_library(tukamukai_solver src/solver.cpp)
target_include_directories(tukamukai_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# マスの座標をケース座標・ロボット座標に直す（ROS に依存しないライブラリ、#31）
add_library(tukamukai_geometry src/board_geometry.cpp)
target_include_directories(tukamukai_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tukamukai_geometry tukamukai_solver)

# 実行ファイル。ノードを作ったらここに add_executable を追加し、install(TARGETS) にも足す
add_executable(solver_demo src/solver_demo.cpp)
target_link_libraries(solver_demo tukamukai_solver)

# ソルバーの解を RViz に表示する
add_executable(solver_viz src/solver_viz.cpp)
target_link_libraries(solver_viz tukamukai_geometry)
ament_target_dependencies(solver_viz rclcpp geometry_msgs std_msgs visualization_msgs)

# 置く・つかむ動作の確認用ノード（MoveIt）
add_executable(pick_place_node src/pick_place_node.cpp)
ament_target_dependencies(pick_place_node
  angles
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  tf2_geometry_msgs
)
target_link_libraries(pick_place_node tukamukai_geometry)

install(TARGETS tukamukai_solver tukamukai_geometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS solver_demo solver_viz pick_place_node
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

  ament_add_gtest(test_board_geometry test/test_board_geometry.cpp)
  target_link_libraries(test_board_geometry tukamukai_geometry)
endif()

ament_package()
