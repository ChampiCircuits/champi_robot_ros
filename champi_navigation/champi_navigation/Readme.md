# VOCAB
un path c'est la suite de points geometriques
une trajectory ce sont ces points mais avec des points intermediaires, et aussi avec les vitesses et accels qui vont avec (ou pas ???)
une routine c'est aller à un endroit faire qqc, donc contient une position/zone, une action, un nb de points etc.

# NOTES
par dessus tout ca on aura un object planner global qui donne le path pour aller faire une routine
le path est converti en objet Trajectory par le trajectory builder
la Trajectory est donnée au controller qui appelle Trajectory.get_next_poses() pour obtenir les prochains points de passage.
du coup on aurait le robot qui doit suivre une trajectoire de plein de points (au niveau de details qu'on choisira) plutôt que 
juste les control points. 



- avoidance
- gui_v2
- gui
- kinematics models
- math_bind
- path_planner
- pid
- pose_control
- rviz_img_updater
- trajectory
- utils
- world_state
- nav_node