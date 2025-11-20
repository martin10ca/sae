cmake_minimum_required(VERSION 3.8)
project(service_package)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generar el código del servicio
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetSpeed.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
