[toc]
***

# 使用 colcon 构建

```sh
# 先 source 刚才安装的消息包环境
source /home/byd/node/agv_app_msgs/setup.bash

# 然后再构建 server，同样指定它的安装位置
# --install-base: 告诉 colcon 把最终产物放到哪里。
colcon build --packages-select agv_app_server --install-base /home/byd/node/agv_app_server
```