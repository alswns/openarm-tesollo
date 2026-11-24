cmake_minimum_required(VERSION 3.16)
project(openarm_tesollo_moveit)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${generated_dir}")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_hand_xacro.py"
          "${generated_dir}" "${CMAKE_CURRENT_SOURCE_DIR}/../dg5f_s_ros2/dg5f_s_description/urdf"
  RESULT_VARIABLE generation_result
)
if(NOT generation_result EQUAL 0)
  message(FATAL_ERROR "Failed to generate prefixed DG-5F-S Xacro models")
endif()

install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})
install(FILES web/index.html DESTINATION share/${PROJECT_NAME}/web)
install(DIRECTORY web/dist/ DESTINATION share/${PROJECT_NAME}/web/dist)
install(DIRECTORY "${generated_dir}/" DESTINATION share/${PROJECT_NAME}/urdf)
install(PROGRAMS scripts/pose_markers.py scripts/web_pose_bridge.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model test/test_model.py)
  ament_add_pytest_test(test_web_pose_bridge test/test_web_pose_bridge.py)
endif()

ament_package()
