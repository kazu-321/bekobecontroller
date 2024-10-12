cmake_minimum_required(VERSION 3.5)
project(bekobecontroller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(twistring REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Enable automatic MOC, UIC, and RCC processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Add the executable
add_executable(bekobecontroller src/controller.cpp)

# Specify the dependencies for the executable
ament_target_dependencies(bekobecontroller
  rclcpp
  sensor_msgs
  twistring
  cv_bridge
)

# Link libraries
target_link_libraries(bekobecontroller
  ${OpenCV_LIBRARIES}
  Qt5::Widgets
)

# Install the target
install(TARGETS
  bekobecontroller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
