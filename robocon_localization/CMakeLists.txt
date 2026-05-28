cmake_minimum_required(VERSION 3.10)
project(robocon_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(COMMON_DEPS
  cv_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
)

function(add_robocon_node target)
  add_executable(${target} ${ARGN})
  ament_target_dependencies(${target} ${COMMON_DEPS})
  target_link_libraries(${target} ${OpenCV_LIBS})
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_robocon_node(calibrate_dist
  src/calibrate_dist.cpp
)

add_robocon_node(table_dist
  src/table_dist.cpp
  src/utility/distance_lookup.cpp
)

add_robocon_node(hsv_adjust
  src/hsv_adjust.cpp
)

add_robocon_node(localization
  src/localization.cpp
  src/utility/distance_lookup.cpp
  src/utility/lines_map.cpp
)

add_robocon_node(load_lines_map
  src/load_lines_map.cpp
  src/utility/lines_map.cpp
)

add_robocon_node(omni_mirror_sim
  src/omni_mirror_sim.cpp
)

add_robocon_node(loc_sidelines
  src/loc_sidelines.cpp
  src/utility/distance_lookup.cpp
  src/utility/lines_map.cpp
  src/utility/lines_matcher.cpp
)

add_robocon_node(relocalization
  src/relocalization.cpp
  src/utility/distance_lookup.cpp
  src/utility/lines_map.cpp
  src/utility/lines_matcher.cpp
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
