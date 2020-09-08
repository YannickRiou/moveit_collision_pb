# moveit_collision_pb

# Step to reproduce 

1. Launch pr2.launch located in pr2_test package . It will launch moveit, rviz and the node pr2_pb_test.
2. In the terminal where pr2.launch was launched, type a random letter then hit enter. PR2 will move arms to home, open left gripper, then move left to object, attach it, then move into box. 


# Expected behavior

Solution is found for the place when planning and execution shows no collision.

# Actual behavior

Solution is found when planning, we see a collision between the grasped object and the side of the box when clicking on the MTC solution. Execution confirms that, saying a collision has been found. 
When executing, moveit reports a collision between grasped object and scene object.
