cmake_minimum_required(VERSION 3.5)
project(order_py_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Order.action"
  "srv/GetQueuedOrders.srv"
  "srv/UpdateOrderStatus.srv"
  DEPENDENCIES builtin_interfaces
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
