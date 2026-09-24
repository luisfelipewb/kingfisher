cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(kingfisher_description)
find_package(ament_cmake REQUIRED)

# Hooks: expose share/ on GZ_SIM_RESOURCE_PATH so model://kingfisher_description/... resolves
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
