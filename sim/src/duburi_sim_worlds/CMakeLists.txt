cmake_minimum_required(VERSION 3.12.2)
project(duburi_sim_worlds)

find_package(ament_cmake REQUIRED)

# Put models/ and worlds/ on GZ_SIM_RESOURCE_PATH.
ament_environment_hooks(hooks/setup.dsv)

install(DIRECTORY models worlds courses templates scripts spec
        DESTINATION share/${PROJECT_NAME})

ament_package()
