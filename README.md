# SmartRescue ROS Package Development


## Tutorials
    roslaunch rocopar_bringup complete_multi_stage.launch


## Todos


1. 在动态构建的roadmap中不断在线生成任务分配方案；

2. 在迷宫or洞穴场景中边探索边救援；




## Debug
1. move_base 效果差的问题
    > 修改膨胀层参数
    > rocopar_bringup/cfg/diff/global_costmap_params.yaml中inflation_radius修改为0.2
    > rocopar_bringup/cfg/diff/local_costmap_params.yaml中inflation_radius修改为0.1