# 工作计划

Author: Andy

Last Update: 2024.08.10

## 阶段一计划（Done)

### map_server 节点

1. 读取已有地图，发布/map话题（OC map)

### roadmap_builder 节点

1. 接受\map话题并储存（桩模块)
2. 调用\gen_voronoi服务，请求包含occupancy map，回复为voronoi map（tuw_multi_robot_msgs/Graph）

### voronoi_generator 节点

1. 把当前/map的话题回调和segments的循环发布，修改为服务器机制

### Others

1. 识别tuw_msgs中使用的模块，删除无关模块

## 阶段二计划
