# moveit_collision_pb

# Step to reproduce 

Launch pr2.launch located in pr2_test package . It will launch moveit, rviz and the node pr2_pb_test that move an attached object into a "box". 

# Expected behavior

Solution is found when planning and execution shows no collision 

# Actual behavior

Solution is found when planning, we see a collision between the grasped object and the side of the box or the support table when clicking on the MTC solution. Execution confirms that, saying a collision has been found. 

Here is a video of the found behavior : 
https://catdrop.drycat.fr/r/u-xsrDL1#9JGJ/BsH/1hnOuoKNPn2vP1ioNnQMuhckpf8107xjn8=

As RRT is a random planner, it sometimes manage to find a solution without a collision.

