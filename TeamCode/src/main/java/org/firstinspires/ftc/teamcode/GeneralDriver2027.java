package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.hardware.Hardware2026;
import org.firstinspires.ftc.teamcode.hardware.Hardware2027;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2023;

@TeleOp(name="Driver2027", group="TeleOps")
public class GeneralDriver2027 extends LinearOpMode {

    Hardware2027 hdw;
    MecanumWheels2023 robotWheel;

    double turbo = 1.0;

    //Set the max power scale.  Max is 1.
    double MAX_LAUNCH_POWER = 0.8;


    @Override
    public void runOpMode() {

        hdw = new Hardware2027(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2023();

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        Gamepad currentGamePad1 = new Gamepad();
        Gamepad previousGamePad1 = new Gamepad();

        currentGamePad1.copy(gamepad1);

        while (opModeIsActive()) {
            //Record previous Gamepad Status
            previousGamePad1.copy(currentGamePad1);
            //Update current gamepad status
            currentGamePad1.copy(gamepad1);

            robotWheel.joystick(gamepad1, turbo);

            hdw.wheelFrontLeft.setVelocity(robotWheel.wheelFrontLeftPower * Hardware2027.ANGULAR_RATE);
            hdw.wheelBackLeft.setVelocity(robotWheel.wheelBackLeftPower * Hardware2027.ANGULAR_RATE);
            hdw.wheelFrontRight.setVelocity(robotWheel.wheelFrontRightPower * Hardware2027.ANGULAR_RATE);
            hdw.wheelBackRight.setVelocity(robotWheel.wheelBackRightPower * Hardware2027.ANGULAR_RATE);

            hdw.setLuncherPower( (float) (currentGamePad1.left_trigger * MAX_LAUNCH_POWER) );
            hdw.setIntakePower(currentGamePad1.right_trigger);

            if (!previousGamePad1.left_bumper && currentGamePad1.left_bumper) {
                hdw.intakeSideWheelsOn();
            }
            if (previousGamePad1.left_bumper && !currentGamePad1.left_bumper) {
                hdw.intakeSideWheelsOff();
            }

            if (!previousGamePad1.right_bumper && currentGamePad1.right_bumper) {
                hdw.innerOn();
            }
            if (previousGamePad1.right_bumper && !currentGamePad1.right_bumper) {
                hdw.innerOff();
            }

        }

    }
}
