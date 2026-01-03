package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.AnchorRunnable;
import org.firstinspires.ftc.teamcode.hardware.ArmControlRunable;
import org.firstinspires.ftc.teamcode.hardware.CarouelController;
import org.firstinspires.ftc.teamcode.hardware.Hardware2026;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2023;

public abstract class GeneralDriver2026 extends LinearOpMode {
    int targetTagId = 0;

    Hardware2026 hdw;

    MecanumWheels2023 robotWheel;
    CarouelController car;
    AnchorRunnable anchorRunnable = null;

    float autoLaunchPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Don't initialize carousel position.  Make sure it's aligned
        car = new CarouelController(hardwareMap, telemetry);
        car.initialize();
        hdw = new Hardware2026(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2023();

        double turbo = 1;

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        Gamepad currentGamePad1 = new Gamepad();
        Gamepad previousGamePad1 = new Gamepad();

        currentGamePad1.copy(gamepad1);

        anchorRunnable = new AnchorRunnable(hdw,this);
        Thread anchorThread = new Thread(anchorRunnable);
        anchorThread.start();
        Log.d("9010", "new thread running");

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

            hdw.setIntakePower(currentGamePad1.left_trigger);
            if ( autoLaunchPower==0 ) {
                car.setLauncherPower(currentGamePad1.right_trigger * (float) 0.95);
            }

            if (gamepad1.back) {
                car.initPosition();
            }
            if (!previousGamePad1.ps && currentGamePad1.ps) {
                robotWheel.setHeadingForward(!robotWheel.isHeadingForward());
                telemetry.addLine().addData("Heading Foward: ", robotWheel.isHeadingForward());
                telemetry.update();
            }

            if (!previousGamePad1.x && currentGamePad1.x) {

            }

            if (!previousGamePad1.y && currentGamePad1.y) {
            }


            if (currentGamePad1.a) {
                if ( hdw.getIntakePower()!=0 ) {
                    hdw.setIntakePower(0);
                } else {
                    hdw.setIntakePower(-1 * hdw.INTAKE_POWER);
                }
            }

            if (currentGamePad1.b && ! previousGamePad1.b) {
                if (anchorRunnable.isRunning()) {
                    anchorRunnable.setRunning(false);
                } else {
                    anchorRunnable.setRunning(true);
                }
                telemetry.addLine().addData("Anchor running: ",anchorRunnable.isRunning());
                telemetry.update();
            }

            if (!previousGamePad1.right_stick_button && currentGamePad1.right_stick_button) {
                autoLaunchPower = (float) 0.95;
                car.setLauncherPower(autoLaunchPower);
                hdw.moveByAprilTag(this.targetTagId, 1900);
            }

            if (!previousGamePad1.left_stick_button && currentGamePad1.left_stick_button) {
                car.shootBall();
                sleep(500);
                autoLaunchPower = 0;
            }

            if ( currentGamePad1.dpad_left){
                car.rotateOneSlotCCW();
            }
            if (currentGamePad1.dpad_right) {
                car.rotateOneSlotCW();
            }

            if ( currentGamePad1.left_bumper){
                //Shooting green ball
            }
            if (currentGamePad1.right_bumper) {
                //Shooting purple ball
            }
        }

    }
}
