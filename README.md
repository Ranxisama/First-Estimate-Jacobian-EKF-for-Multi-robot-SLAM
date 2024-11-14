# Project name: MultiRobotsEKF
This project compares the First Eitimate Jacobian Extended Kalman Filter (FEJ-EKF) with the ideal Extended Kalman Filter (Ide-EKF) and the standard Extended Kalman Filter (Std-EKF) for Multi-robot Simultaneous Localization and Mapping (SLAM).
The FEJ-EKF is designed to improve the filter performance in the presence of non-linearities in the robot motion and sensor models.

## Main file structure and instruction
```
MultiRobotsEKF/
1. Open and run Config.m to configure the parameters for the simulation and the Vicktoria Park dataset based experiment;
2. Open and run MonteCarloMeaGenerate.m to generate and save the data of parameters and measurements for the simulation and the Vicktoria Park dataset based experiment;
3.  (a) Open and run MonteCarloExp_FEJ.m to implement the FEJ-EKF for the simulation or the Vicktoria Park dataset based experiment using the data generated from MonteCarloMeaGenerate.m and save the result data;
    (b) Open and run MonteCarloExp_Ide.m to implement the Ide-EKF for the simulation using the data generated from MonteCarloMeaGenerate.m and save the result data;
    (c) Open and run MonteCarloExp_Std.m to implement the Std-EKF for the simulation and the Vicktoria Park dataset based experiment using the data generated from MonteCarloMeaGenerate.m and save the result data;
4. Open and run MTExpPLOT.m to:
    1. Plot and save 3-sigma bound, RMSE and NEES of a specific configured scenairo using the data generated from MonteCarloExp_FEJ.m, MonteCarloExp_Ide.m and MonteCarloExp_Std.m;
    2. Display the average RMSE and the average NEES in Command Window.

## Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/Ranxisama/First-Estimate-Jacobian-EKF-for-Multi-robot-SLAM.git
