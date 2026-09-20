cmake_minimum_required(VERSION 3.12.2)
project(mongla_sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mongla_sim DESTINATION lib/${PROJECT_NAME})

ament_package()
