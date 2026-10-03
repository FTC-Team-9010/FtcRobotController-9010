## Get Started 
2026.10.1 
To start, please download Android studio,  Version to use for 2026 - 2027 season is Rabbit version. 
Download can be found https://developer.android.com/studio?utm_source=android-studio 

The github code repo for this code is here: https://github.com/FTC-Team-9010/FtcRobotController-9010 
No speical access is required to view the code. 
However, to be able to modify the code, please register a user on Github, and apply for access for 
this Repository.  Out team manager's account is 

## Code Structure 
Our team code is in FtcRobotController-9010\TeamCode\ directory.

* TeamCode/src/main/java/org/firstinspires/ftc/teamcode  
This directory includes the TeleOp class for the robot control, as well as autonomous Op code 

Here are some example of code:<br> 
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/GeneralDriver2026.java  This file is our code for the driver control. <br>
Inside this main loop is the acction for each botton of controller, how robot shall act . 
<br>
<code>        while (opModeIsActive()) {
</code><br>

This Class have the logic how to control 2025-2026 year robot, except carouel, which is in another class.   
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/Hardware2026.java  

This class shows the how caruel control is done.  
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/CarouelController.java

* TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware 
This directory includes code represent our robot hardware control.  These code are closed realted to the hardware of robot design. E.g. Ports connection,  sensors, motors etc.  

## Reference: 
First provided resources for the programing, here listed some:
https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html <br>
Please Note that we are using Android Studio as programing tool, some of article is based on OnBot Or Block programing. Please skip those sections. 

