--colcon build 编译

--source install/setup.zsh（若是bash则修改成install/setup.bash）

--ros2 launch x100_moveit_config demo.launch.py  启动moveit2和rviz

--若无需可视化可使用以下命令
  ros2 launch x100_moveit_config demo.launch.py use_rviz:=false

--ros2 launch x100_motion_planning x100_motion_planning.launch.py 启动规划结点

PS：规划结点启动时，关于right_hand和left_hand以及other_joints的报错可以忽略
![alt text](<Screenshot from 2025-09-15 18-13-10.png>)