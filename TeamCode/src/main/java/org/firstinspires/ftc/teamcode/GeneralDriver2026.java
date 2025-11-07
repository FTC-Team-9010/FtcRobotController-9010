package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Hardware2026;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2023;

@TeleOp(name="GeneralDriver2026", group="TeleOps")
public class GeneralDriver2026 extends LinearOpMode {

    Hardware2026 hdw;

    MecanumWheels2023 robotWheel;


    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2026(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2023();

        double turbo = .6;

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

            hdw.wheelFrontLeft.setVelocity(robotWheel.wheelFrontLeftPower * Hardware2026.ANGULAR_RATE);
            hdw.wheelBackLeft.setVelocity(robotWheel.wheelBackLeftPower * Hardware2026.ANGULAR_RATE);
            hdw.wheelFrontRight.setVelocity(robotWheel.wheelFrontRightPower * Hardware2026.ANGULAR_RATE);
            hdw.wheelBackRight.setVelocity(robotWheel.wheelBackRightPower * Hardware2026.ANGULAR_RATE);

            if (!previousGamePad1.x && currentGamePad1.x) {
                if ( hdw.getIntakePower()!=0 ) {
                    hdw.setIntakePower(0);
                } else {
                    hdw.setIntakePower(1);
                }
            }
            if (!previousGamePad1.y && currentGamePad1.y) {
                if ( hdw.getLauncherPower()!=0 ) {
                    hdw.setLauncherPower(0);
                } else {
                    hdw.setLauncherPower((float) 0.5);
                }
            }
            
            if (currentGamePad1.a) {
                hdw.raiseLever();
            }
            if (currentGamePad1.b) {
                hdw.lowerLever();
            }


        }

    }
}
