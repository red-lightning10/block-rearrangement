# PDDL Representation of Scenes

Reference document to show the grounded predicates for the given scene in image. 

Note that partial goals are also accepted for planning using pyperplan. By extension, our package (task_planner) will also accept partial goals (just specifiying blue cube on red is enough instead of mentioning about handempty, is_holding, clear, or carrying over current predicates to the goal string)
### Scene 1
---
![Scene 1](images/scene_1.jpg)

**Goal:** (and (ontable blue_cube_0) (ontable red_cube_0) (ontable green_cube_0) (ontable orange_cube_0) (ontable yellow_cube_0) (ontable orange_cube_1))

If you want to replicate **Scene 2** from **Scene 1**, just give the following string as goal:

(and (on green_cube_0 orange_cube_1))

### Scene 2
---
![Scene 2](images/scene_2.jpg)

**Goal:** (and (ontable blue_cube_0) (ontable red_cube_0) (on green_cube_0 orange_cube_1) (ontable orange_cube_0) (ontable yellow_cube_0))

### Scene 3
---
![Scene 3](images/scene_3.jpg)

**Goal:** (and (ontable blue_cube_0) (holding red_cube_0) (on green_cube_0 orange_cube_1) (ontable orange_cube_0) (ontable yellow_cube_0))

### Scene 4
---
![Scene 4](images/scene_4.jpg)

**Goal:** (and (ontable blue_cube_0) (on red_cube_0 yellow_cube_0) (on green_cube_0 orange_cube_1) (ontable orange_cube_0))

### Scene 5
---
![Scene 5](images/scene_5.jpg)

**Goal:** (and (on orange_cube_0 blue_cube_0) (on red_cube_0 yellow_cube_0) (on green_cube_0 orange_cube_1))

If you want to replicate **Scene 6** from **Scene 5**, just give the following string as goal:

(and (on yellow_cube_0 orange_cube_0) (on red_cube_0 yellow_cube_0))

### Scene 6
---
![Scene 6](images/scene_6.jpg)

**Goal:** (and (on red_cube_0 yellow_cube_0) (on yellow_cube_0 orange_cube_0) (on orange_cube_0 blue_cube_0) (ontable blue_cube_0) (on green_cube_0 orange_cube_1) (ontable orange_cube_1) (handempty))

### Scene 7
---
![Scene 7](images/scene_7.jpg)

**Goal:** (and (ontable orange_cube_1) (ontable orange_cube_0) (ontable blue_cube_0) (on yellow_cube_0 green_cube_0) (on red_cube_0 blue_cube_0) (on green_cube_0 orange_cube_1))

### Scene 8
---
![Scene 8](images/scene_8.jpg)

**Goal:** (and (ontable orange_cube_1) (on green_cube_0 orange_cube_1) (on yellow_cube_0 green_cube_0) (on red_cube_0 yellow_cube_0) (on orange_cube_0 red_cube_0) (on blue_cube_0 orange_cube_0))
