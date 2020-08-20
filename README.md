# moveit_collision_pb using MOVEIT

# Step to reproduce 

Launch pr2.launch located in pr2_test package . It will launch moveit, rviz and the node pr2_pb_test_moveit that move an attached object into a "box". 

# Expected behavior

Solution is found when planning and execution shows no collision 

# Actual behavior

Solution is found when planning, we see a collision between the grasped object and the side of the box or the support table when clicking on the MTC solution. Execution confirms that, saying a collision has been found. 

Here is a video of the found behavior : 
https://catdrop.drycat.fr/r/o2qfRMPn#A26Shrh0lsqXxHyvNNinmcIcLliVChCgS1Bek74/6RA=

As RRT is a random planner, it sometimes manage to find a solution without a collision.

