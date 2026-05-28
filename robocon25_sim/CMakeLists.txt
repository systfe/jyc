cmake_minimum_required(VERSION 3.10)
project(robocon25_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch materials meshes models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
