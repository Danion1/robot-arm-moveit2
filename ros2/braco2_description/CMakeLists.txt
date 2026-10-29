cmake_minimum_required(VERSION 3.8)
project(braco2_description)

find_package(ament_cmake REQUIRED)

# Instala as pastas urdf/, meshes/, launch/, rviz/, config/ e worlds/ junto com o pacote,
# para que o ROS2 consiga encontrá-las via package:// depois do colcon build.
install(
  DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
