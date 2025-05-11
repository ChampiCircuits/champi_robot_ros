
# BRAIN
- le nom de la strat choisie doit être paramétrable dans le yaml
- le mode debug de la sm
- l'etat wait for ros_init n'attend pas que le service set_pose soit dispo.

# WEB INTERFACE
- voir ce qu'il faut faire pour l'UI


# ASK ANDRE




[path_planner_node.py-8] [ERROR] [1746735509.010892760] [path_planner.action_server]: Error raised in execute callback: sleep length must be non-negative
[path_planner_node.py-8] Traceback (most recent call last):
[path_planner_node.py-8]   File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/action/server.py", line 336, in _execute_goal
[path_planner_node.py-8]     execute_result = await await_or_execute(execute_callback, goal_handle)
[path_planner_node.py-8]                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
[path_planner_node.py-8]   File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/executors.py", line 108, in await_or_execute
[path_planner_node.py-8]     return await callback(*args)
[path_planner_node.py-8]            ^^^^^^^^^^^^^^^^^^^^^
[path_planner_node.py-8]   File "/home/champi/champi_ws/install/champi_navigation/lib/champi_navigation/path_planner_node.py", line 259, in execute_callback
[path_planner_node.py-8]     time.sleep(self.loop_period - (time.time() - t_loop_start))
[path_planner_node.py-8] ValueError: sleep length must be non-negative
[path_planner_node.py-8] [WARN] [1746735509.015736201] [path_planner.action_server]: Goal state not set, assuming aborted. Goal ID: [246 106 181 253  29  89  68 215 163 240 201 226 255  81 111 123]




[path_planner_node.py-8] [ERROR] [1746809888.504423011] [path_planner.action_server]: Error raised in execute callback: sleep length must be non-negative
[path_planner_node.py-8] Traceback (most recent call last):
[path_planner_node.py-8]   File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/action/server.py", line 336, in _execute_goal
[path_planner_node.py-8]     execute_result = await await_or_execute(execute_callback, goal_handle)
[path_planner_node.py-8]                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
[path_planner_node.py-8]   File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/executors.py", line 108, in await_or_execute
[path_planner_node.py-8]     return await callback(*args)
[path_planner_node.py-8]            ^^^^^^^^^^^^^^^^^^^^^
[path_planner_node.py-8]   File "/home/champi/champi_ws/install/champi_navigation/lib/champi_navigation/path_planner_node.py", line 259, in execute_callback
[path_planner_node.py-8]     time.sleep(self.loop_period - (time.time() - t_loop_start))
[path_planner_node.py-8] ValueError: sleep length must be non-negative
[path_planner_node.py-8] [WARN] [1746809888.527093844] [path_planner.action_server]: Goal state not set, assuming aborted. Goal ID: [184  66  33 133 113  42  75  14 128 148 224 171 190  53  49 137]
