In order to run the follownig codes, ensure that matlab is installed on the machine, with the "ROS Toolbox" and "Robotics System Toolbox" added as well.

Exercise 1:
The zip folder was downloaded and the matlab file "Test_CDMP.m" was opened and edited. Following changes were made to the code.

At line 15: A function was made in order to calculate the joint trajectories. Requiring the DMP parameters from the xx file as input, and it would give the Joint DMP parameters as output. The function was made as a seperate file, similarly to the QDMP function. The function starts out by setting the DMP parameters, and then inserting them into the DMP_rlearn function, using the path and the current DMP parameters as input.

The DMP_rlearn.m file were edited to include following variables and formulas. The initial and goal state of the DMP was added. Then a formular to derive the trajectory was made in order to obtain the velocity, and was divided with the sampling rate to get xx. Those results were then dirived again to obtain the acceleration, which was also divided with the sampling rate. The next addition was a function to calculate the weighted sum of the locally weighted regression models, in order to xx. Next was a temporal scaling derivative which was divided with the torgue, in order to xx. Lastly for the function file, a second order DMP equation was added in order to fit the target to the DMP line. It was done by xx. 

Back to the Test_CDMP.m file, an intergration was done, similarly to the position part of the DMP, in order to get the joint positions.

Lastly, the DMP was plotted into three different figures, one displaying position, another velocity, and the last displaying acceleration over the trajectory obtained from the test_trj.mat file.
