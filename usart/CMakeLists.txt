cmake_minimum_required(VERSION 3.5)
project(usart)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}/..")


# set(CMAKE_CXX_STANDARD 17)
# add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(fmt REQUIRED)  
# find dependencies
find_package(FMT REQUIRED)
find_package(serial REQUIRED)
# target_link_libraries(your_program_name fmt::fmt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

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


INCLUDE_DIRECTORIES(
include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#导入tool库
include_directories(
/usr/local/include
)



add_executable(virtual_usart src/virtual_serialport.cpp)

# add_executable(usart src/usart.cpp src/receive.cpp src/send.cpp)


ament_target_dependencies(virtual_usart
rclcpp 
interfaces
OpenCV
FMT
Eigen3
tf2 tf2_eigen tf2_ros
)


install(TARGETS
virtual_usart
DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
config
DESTINATION share/${PROJECT_NAME}
)

ament_package()