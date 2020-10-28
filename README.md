# moveit_collision_pb -> moveTo anomaly

# Step to reproduce 

$ git clone https://github.com/YannickRiou/moveit_collision_pb.git
$ cd moveit_collision_pb/
$ git checkout moveTo
$ git submodule init
$ git submodule update

$ roslaunch pr2_test pr2.launch fake_execution:=true
 

# Expected behavior

Predefined pick and place occurs and object is still in the scene and detached from the robot at the end of the execution

# Actual behavior

Predefined pick and place occurs and object disappear at the end of the "moveTo end" stage, the behavior is confirmed by the error saying that the detach stage can find "obj_230" in the scene. The object is seen disappearing also in the planned path.

Here is the gist of the run : https://gist.github.com/YannickRiou/951baf4dc6f2598058c804d8eeedc251

