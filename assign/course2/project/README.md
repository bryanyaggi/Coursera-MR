# Course 2 (Robot Kinematics) Project

## Contents
- `code` is where all code is located. `project.py` implements
`IKinBodyIterates` and solves the UR5 joint values for the desired
configuration. `project.sh` is a shell script that runs `project.py` and writes
the console output to `log.txt`.
- `log.txt` contains the console output.
- `iterates.csv` contains the joint values for each iteration.
- `screenshot.png` shows the final robot configuration in CoppeliaSim.
- `animation.avi` is a video of the robot moving to the final configuration via
the iterative Newton-Raphson solutions in CoppeliaSim.
