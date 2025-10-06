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
（尽量还是第一种吧）

##如果BaristaBot卡住


订单会卡在IN——PROGRESS状态，状态无法被更新

处理方法：

设置一个订单监控器，定时监测订单是否在IN_PROGRESS卡住超时15秒，
如果确实存在卡住的异常，就启动订单重置，
先保持原来的订单号不变，然后将订单状态重新改为QUEUED，之后重新被Baristabot获取并处理订单






![97ba6517c3bb01ab119b97d38aad8c48](https://github.com/user-attachments/assets/9185fa43-20de-43fd-9e55-144d39e391c7)



