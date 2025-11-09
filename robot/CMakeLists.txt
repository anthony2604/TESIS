cmake_minimum_required(VERSION 3.5)  # ROS 2 requiere al menos CMake 3.5

project(robot)

# Usamos ament como el sistema de construcción
# Se debe especificar que las dependencias son parte de ROS 2
find_package(ament_cmake REQUIRED)

# Paquetes de ROS 2 requeridos, eliminamos 'catkin' y usamos 'ament' y 'ros2' en su lugar
find_package(rclcpp REQUIRED)  # Usamos rclcpp en lugar de roscpp
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)


# Si necesitas otros paquetes, agrega aquí 'find_package' para ellos

# Si tienes dependencias adicionales de tipo 'message generation' puedes gestionarlo aquí también
# Para mensajes personalizados en ROS 2, debes usar 'rosidl' (ROS Interface Definition Language)
# find_package(rosidl_default_generators REQUIRED)

# Aquí puedes definir la instalación de los recursos del robot, como 'urdf' y 'launch'
install(
  DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)


# Al final, asegúrate de invocar a ament para completar el proceso de construcción
ament_package()


