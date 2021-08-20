cmake_minimum_required(VERSION 3.0.2)
project(mcl2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ui/)
#set(Qt5_DIR "/usr/lib/aarch64-linux-gnu/cmake/qt5")
set(Qt5_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5")
#set(OpenCV_DIR "/usr/local/share/OpenCV")

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  roscpp
)

set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

set(sources_srcs
    src/mcl_viewer.cpp
    src/mainwindow.cpp
    src/mcl_item.cpp
    src/mcl.cpp
    src/control.cpp
)

set(sources_hpps
    include/mcl_localization/control.h
    include/mcl_localization/mainwindow.h
    include/mcl_localization/mcl.h
    include/mcl_localization/mcl_item.h
    include/mcl_localization/mcl_properties.h
)

set(sources_uis
    ui/control.ui
    ui/mainwindow.ui
)

qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${sources_hpps} ${sources_srcs})
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${sources_uis})

include_directories(
    include
    include/mcl_localization
    ${YAML_CPP_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIRS}
    )


add_executable(${PROJECT_NAME} ${sources_srcs} ${${PROJECT_NAME}_MOCS} ${sources_uis} ${${PROJECT_NAME}_UIS_H})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES}
    Qt5::Gui
    Qt5::Core
    Qt5::Widgets
    )

add_message_files(
    FILES
    mcl2_local_result.msg
    mcl2_sensor_data.msg
  )

generate_messages(
    DEPENDENCIES
    std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES mcl2
#  CATKIN_DEPENDS rospy std_msgs
   CATKIN_DEPENDS std_msgs message_runtime roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(mcl_local mainwindow.cpp)
add_dependencies(mcl2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mcl2 ${catkin_LIBRARIES})



