#order_py_pkg

##启动和运行

新建两个终端，目录为order_py_pkg所在的ros2_ws目录

*bash*
colcon build

然后分别运行：

source install/setup.bash
ros2 run order_py_pkg OrderManager_node

source install/setup.bash
ros2 run order_py_pkg BaristaBot_node

接着提交测试订单：

ros2 action send_goal/submit_order order_py_pkg/Order"{order_items:['latte','cappuccino']}

或者也可以：

新建第三个终端，并启动顾客节点Customer_node

![97ba6517c3bb01ab119b97d38aad8c48](https://github.com/user-attachments/assets/9185fa43-20de-43fd-9e55-144d39e391c7)



