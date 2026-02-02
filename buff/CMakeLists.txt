cmake_minimum_required(VERSION 3.8)
project(buff)

# 标准配置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-fopenmp -DEIGEN_NO_DEBUG)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/buff
  ${CMAKE_CURRENT_SOURCE_DIR}/include/buff/utils
  ${CMAKE_CURRENT_SOURCE_DIR}/include/buff/optimization
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


# 依赖查找
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(interfaces REQUIRED)

# 组件库定义
add_library(rune_preprocessor_component SHARED
  src/rune_preprocessor.cpp
  src/utils/pnp_solver.cpp
)
set_source_files_properties(src/utils/pnp_solver.cpp 
    PROPERTIES COMPILE_OPTIONS "-O3")

target_compile_definitions(rune_preprocessor_component PRIVATE "buff_COMPONENT_BUILDING")
target_link_libraries(rune_preprocessor_component
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
ament_target_dependencies(rune_preprocessor_component
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_eigen
  geometry_msgs
  visualization_msgs
  OpenCV
  Ceres
  Eigen3
  interfaces
)
rclcpp_components_register_nodes(rune_preprocessor_component PLUGIN "rune::RunePreprocessor" EXECUTABLE RunePreprocessor)

add_library(rune_modeling_component SHARED
  src/rune_modeling.cpp
  src/utils/rune_fan_defination.cpp
)

target_compile_definitions(rune_modeling_component PRIVATE "buff_COMPONENT_BUILDING")
target_link_libraries(rune_modeling_component
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
ament_target_dependencies(rune_modeling_component
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_eigen
  visualization_msgs
  geometry_msgs
  OpenCV
  Ceres
  Eigen3
  interfaces
)
rclcpp_components_register_nodes(rune_modeling_component PLUGIN "rune::RuneModeling" EXECUTABLE RuneModeling)

add_library(rune_fire_controller_component SHARED
  src/rune_fire_controller.cpp
  src/utils/ballistic_solver.cpp
)
set_source_files_properties(src/utils/ballistic_solver.cpp 
    PROPERTIES COMPILE_OPTIONS "-O3")

target_compile_definitions(rune_fire_controller_component PRIVATE "buff_COMPONENT_BUILDING")
target_link_libraries(rune_fire_controller_component
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
ament_target_dependencies(rune_fire_controller_component
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_eigen
  geometry_msgs
  visualization_msgs
  OpenCV
  Ceres
  Eigen3
  interfaces
)
rclcpp_components_register_nodes(rune_fire_controller_component PLUGIN "rune::RuneFireController" EXECUTABLE RuneFireController)





#   rune test package
# find dependencies
find_package(CURL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
add_executable(virtual_rune src/rune_test.cpp src/utils/ballistic_solver.cpp)

target_compile_features(virtual_rune PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(virtual_rune
  Eigen3
  OpenCV
  visualization_msgs
  "rclcpp"
  "interfaces"
  "cv_bridge"
  tf2 tf2_eigen tf2_ros
)

target_link_libraries(virtual_rune
        ${OpenCV_LIBS}
        ${CURL_LIBRARIES}
        ${CERES_LIBRARIES}
        )
install(TARGETS virtual_rune
  DESTINATION lib/${PROJECT_NAME})


# 安装规则
ament_export_targets(export_rune_preprocessor_component)
install(TARGETS rune_preprocessor_component
        EXPORT export_rune_preprocessor_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_export_targets(export_rune_modeling_component)
install(TARGETS rune_modeling_component
        EXPORT export_rune_modeling_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_export_targets(export_rune_fire_controller_component)
install(TARGETS rune_fire_controller_component
        EXPORT export_rune_fire_controller_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()