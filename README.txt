For the extra credit writing robot that we've implemented, please open and run the ur5_project_EC.m script. 

Prior to running, there are a few parameters that should be adjusted
1. The actual phrase being written
   (This can be any combination of lower case english letters, as well as spaces.
    The phrase/word that you run will be scaled accoringly to be able to fit in the
    UR5 workspace.)
2. The mode of writing ('rviz'/'mplot')
    There are 2 modes for the drawing:
    1. 'rviz' actually runs the simulation with the UR5 robot.
       Note that for long phrases/words (7+ letters), this can take a very long time
    2. 'mplot' plots the planned path in a matlab 2D plot. It will also show how the writing would be positioned wrt to the robot's workspace.
3. K gain
    The gain value for the RRcontrol scheme used in the robot. Default is 0.75.

The script will create the robot object, and draw the word using the mode designated.

It is recommended to set the 'Marker Scale' in rviz to be 0.05, for the written letters in rviz to be reable.