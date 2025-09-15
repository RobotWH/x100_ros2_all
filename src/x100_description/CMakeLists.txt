cmake_minimum_required(VERSION 3.5)
project(x100_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf mesh
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
