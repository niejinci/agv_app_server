[toc]
***

# 使用 colcon 构建

```sh
rm -rf build/agv_app_server install/agv_app_server

# 先 source 刚才安装的消息包环境
source /home/byd/node/agv_app_msgs/setup.bash

# 然后再构建 server，同样指定它的安装位置
# --install-base: 告诉 colcon 把最终产物放到哪里。
colcon build --packages-select agv_app_server --install-base /home/byd/node/agv_app_server
```

# 调试

## 本地编译

```sh
# 进入工作区
cd ~/njc_ws/

# 清理旧的（可选，推荐）
rm -rf build/agv_app_server install/agv_app_server

# 更新工作区的代码
cp -rf /mnt/d/byd_agv_njc/agv_app_server ./src/

# 编译
colcon build --packages-select agv_app_server --install-base /home/byd/node/agv_app_server
```

## 查看进程

```sh
ps -ef | grep agv_app_server
```

## 杀掉进程

```sh
kill `pidof agv_app_server_node`
```

## 运行进程

```sh
/home/byd/node/agv_app_server/bin/agv_app_server_node
```