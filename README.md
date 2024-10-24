# SmartRescue ROS Package Development


## Tutorials
    roslaunch rocopar_bringup complete_multi_stage.launch


## Todos
1. 写一个分层的运动规划器，分为global_planner 和local_planner
2. global_planner: 订阅roadmap话题，将其转化为networkx中的Graph，然后可以直接在上面用dijkstra方法得到最短路径
3. local_planner: 将豪泽的规划器引入进来，不断地将global path中的点依次发布

clone RoCoPAR里面最新的内容




## Debug
1. move_base 效果差的问题
    > 修改膨胀层参数
    > rocopar_bringup/cfg/diff/global_costmap_params.yaml中inflation_radius修改为0.2
    > rocopar_bringup/cfg/diff/local_costmap_params.yaml中inflation_radius修改为0.1