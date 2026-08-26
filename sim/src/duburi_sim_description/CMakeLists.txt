cmake_minimum_required(VERSION 3.12.2)
project(duburi_sim_description)

find_package(ament_cmake REQUIRED)

# Put models/ on GZ_SIM_RESOURCE_PATH so model:// URIs resolve.
ament_environment_hooks(hooks/setup.dsv)

install(DIRECTORY models scripts DESTINATION share/${PROJECT_NAME})

ament_package()
