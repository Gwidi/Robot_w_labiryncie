Należy uruchomić stack nawigacyjny:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
oraz wykonać launch algorytmu przeszukiwania labiryntu oraz SLAM_toolbox:
```bash
ros2 launch maze_explorer maze_explorer_launch.py
