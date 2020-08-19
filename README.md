# moveit_collision_pb

# Step to reproduce 

Launch pr2.launch located in pr2_test package . It will launch moveit, rviz and the node pr2_pb_test that move an attached object into a "box". 

# Expected behavior

Solution is found when planning and execution shows no collision 

# Actual behavior

Solution is found when planning, we see a collision beteen the grasped object and the side of the box when clicking on the MTC solution. Execution confirms that saying a collision has been found. 


