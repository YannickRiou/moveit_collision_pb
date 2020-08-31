# moveit_collision_pb

# Step to reproduce 

1. Launch pr2_real.launch located in pr2_test package . It will launch moveit, rviz and the node pr2_tasks.
2. On Rviz, Import the scene located into pr2_test/scene/problematicScene.scene and don't forget to click on "Publish".
3. In the terminal where pr2_real.launch was launched, type a random letter then hit enter, it will begin the pick and place planning (pick left, move left to home, pick right, move right to home, place left). Execution is automatically triggered for all movement except the last one (place left). User needs to enter a random letter into the terminal then press enter to trigger execution of this last task.
 

# Expected behavior

Solution is found for the place when planning and execution shows no collision.

# Actual behavior

Solution is found when planning, we see a collision between the grasped object and the side of the box or the support table when clicking on the MTC solution (place_left). Execution confirms that, saying a collision has been found. 
When executing, moveit reports a collision between grasped object and scene object for the "place_left" task.

Video of the full execution with problems can be downloaded here : https://drop.infini.fr/r/rhAueKWvPz#RTYiVoFPqQNNQQg0jtqFUlyN57CPbgP/KyGi3KwDPUA=
