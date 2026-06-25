cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc-13")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-13")

project(dv_processing_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_path(LIBUSB_INCLUDE_DIR
  NAMES libusb.h
  PATH_SUFFIXES "include" "libusb" "libusb-1.0")
find_library(LIBUSB_LIBRARY
  NAMES usb
  PATH_SUFFIXES "lib" "lib32" "lib64")

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dv-processing REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pugixml REQUIRED)
# find_package(libusb REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/include/**")
#include_directories("$ENV{EIGEN3_INCLUDE_DIR}")


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Event.msg"
  "msg/EventArray.msg"
  DEPENDENCIES 
  "builtin_interfaces"
  "std_msgs"
)

file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

add_executable(dv_processor_node src/main.cpp ${SOURCES})
# target_compile_features(dv_processor_node PRIVATE cxx_std_17)
ament_target_dependencies(dv_processor_node 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_msgs
  # dv-processing
  # libusb
)


install(TARGETS
    dv_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(dv_processor_node "${cpp_typesupport_target}")

# target_include_directories(dv_processor_node ${LIBUSB_INCLUDE_DIR})
target_link_libraries(dv_processor_node Eigen3::Eigen ${LIBUSB_LIBRARY} dv::processing)

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
