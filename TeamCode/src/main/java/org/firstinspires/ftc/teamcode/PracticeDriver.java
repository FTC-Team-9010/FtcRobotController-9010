package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.HardwarePractice;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2023;

@TeleOp(name="PracticeDriver", group="TeleOps")
public class PracticeDriver extends LinearOpMode {

    HardwarePractice hdw;

    MecanumWheels2023 robotWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new HardwarePractice(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2023();
        hdw.armInit();

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        double turbo = 1;

        Gamepad currentGamePad1 = new Gamepad();
        Gamepad previousGamePad1 = new Gamepad();

        currentGamePad1.copy(gamepad1);

        //This is the main loop of operation.
        while (opModeIsActive()) {

            previousGamePad1.copy(currentGamePad1);
            currentGamePad1.copy(gamepad1);

            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%

            robotWheel.joystick(gamepad1, turbo);

            hdw.wheelFrontLeft.setVelocity(robotWheel.wheelFrontLeftPower * HardwarePractice.ANGULAR_RATE);
            hdw.wheelBackLeft.setVelocity(robotWheel.wheelBackLeftPower * HardwarePractice.ANGULAR_RATE);
            hdw.wheelFrontRight.setVelocity(robotWheel.wheelFrontRightPower * HardwarePractice.ANGULAR_RATE);
            hdw.wheelBackRight.setVelocity(robotWheel.wheelBackRightPower * HardwarePractice.ANGULAR_RATE);

            if (!previousGamePad1.y && currentGamePad1.y) {
                if (hdw.getLauncherPower() != 0) {
                    hdw.setLauncherPower(0);
                    hdw.setFlywheelPower(0);
                } else {
                    hdw.setLauncherPower(hdw.practicePresetLaunchPower);
                    hdw.setFlywheelPower(hdw.practiceFlywheelPower);
                }
            }

            if (!previousGamePad1.x && currentGamePad1.x) {
                hdw.armLaunch();
            }

            if (previousGamePad1.x && !currentGamePad1.x) {

                hdw.armInit();
            }


        }

    }
}
