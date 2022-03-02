# Robot-2022

2022 Geneva Robovikes code for the 2022 FRC season. Proceed with caution...

## **How to Run the Robot**
1) Connect to the radio over wifi*, which should end in 3067.
2) Open driver station and verify the connection box is green.
3) Using VS Code, click on the "W" in the top right then select "Deploy Robot Code".
4) Once the code is deployed, go back to driver station and wait for the text, "No Robot Code", to switch to "Disabled".
5) Check to make sure you are on the correct robot mode.
6) Press "Enable" to start the Robot!

If things get a bit spicy, press space for an emergency stop. To restart the robot after, press the restart button on the roboRIO.

*You do not need to wait to connect, as soon as you hit the button you are good.

## **How to make a Pathweaver path**
1) Open Pathweaver 2022.
2) Open the "PathWeaver" project in the "Robot-2022" repository.
3) Press the "+" button in the bottom right.
4) Drag points to move them or click on the orange line to add a point.
5) Drag on the blue line to change the point's angle.
6) Mess around until you get a desirable* path.
7) Press build paths in the bottom right to add changes to the code.
8) Ensure the file name in the "RobotContainer" file is the name of the new path.
9) Redeploy the code as seen in the above section.
10) Enable the Robot in autonomous to see the path in action!

If you want a specific point, type values into the fields at the bottom.

**ALL VALUES ARE IN METERS!** It should look wrong if you put things in a different unit.

*Pathweaver does **NOT** run sharp corners well. Make sure all trajectories are well rounded.
