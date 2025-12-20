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

@TeleOp(name="GeneralDriver2026", group="TeleOps")
public class GeneralDriver2026 extends LinearOpMode {

    Hardware2026 hdw;

    MecanumWheels2023 robotWheel;
    CarouelController car;
    AnchorRunnable anchorRunnable = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //Don't initialize carousel position.  Make sure it's aligned
        car = new CarouelController(hardwareMap, telemetry);
        car.initialize();
        hdw = new Hardware2026(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2023();

        anchorRunnable = new AnchorRunnable(hdw,this);
        Thread armThread = new Thread(anchorRunnable);
        armThread.start();
        Log.d("9010", "new thread running");

        double turbo = 1;

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

            //Log.d("9010", "lfWheel Power:" + robotWheel.wheelFrontLeftPower);
            //Log.d("9010", "lrWheel Power:" + robotWheel.wheelFrontRightPower);
            //Log.d("9010", "lfWheel Velocity:" + robotWheel.wheelFrontLeftPower*Hardware2026.ANGULAR_RATE);

            hdw.wheelFrontLeft.setVelocity(robotWheel.wheelFrontLeftPower * Hardware2026.ANGULAR_RATE);
            hdw.wheelBackLeft.setVelocity(robotWheel.wheelBackLeftPower * Hardware2026.ANGULAR_RATE);
            hdw.wheelFrontRight.setVelocity(robotWheel.wheelFrontRightPower * Hardware2026.ANGULAR_RATE);
            hdw.wheelBackRight.setVelocity(robotWheel.wheelBackRightPower * Hardware2026.ANGULAR_RATE);

            if (!previousGamePad1.x && currentGamePad1.x) {
                if ( hdw.getIntakePower()!=0 ) {
                    hdw.setIntakePower(0);
                } else {
                    hdw.setIntakePower(hdw.INTAKE_POWER);
                }
            }

            if (!previousGamePad1.y && currentGamePad1.y) {
                if ( car.getLauncherPower()!=0 ) {
                    car.setLauncherPower(0);
                } else {
                    car.setLauncherPower(car.presetLaunchPower);
                }
            }

            if (gamepad1.back) {
                car.initPosition();
            }

            if (currentGamePad1.a) {
                if ( hdw.getIntakePower()!=0 ) {
                    hdw.setIntakePower(0);
                } else {
                    hdw.setIntakePower(-1 * hdw.INTAKE_POWER);
                }
            }

            if (currentGamePad1.b && ! previousGamePad1.b) {
                if (robotWheel.isHeadingForward()) {
                    robotWheel.setHeadingForward(false);
                } else {
                    robotWheel.setHeadingForward(true);
                }
            }

            if (currentGamePad1.x && ! previousGamePad1.x) {
                if (anchorRunnable.isRunning()) {
                    anchorRunnable.setRunning(false);
                } else {
                    anchorRunnable.setRunning(true);
                }
                telemetry.addLine().addData("Anchor running: ",anchorRunnable.isRunning());
                telemetry.update();
            }

            if (currentGamePad1.dpad_down) {
                car.shootBall();

            }
            if (currentGamePad1.dpad_up) {

            }
            if ( currentGamePad1.dpad_left){
                car.rotateOneSlotCCW();
            }
            if (currentGamePad1.dpad_right) {
                car.rotateOneSlotCW();
            }

            if ( currentGamePad1.left_bumper){
                car.presetLaunchPower += .1;
                telemetry.addData("Current Launcher Power (0-1): ",car.presetLaunchPower);
                telemetry.update();
                sleep(100);
            }
            if (currentGamePad1.right_bumper) {
                if (car.presetLaunchPower <= 0) {
                    car.presetLaunchPower = 0;
                }
                car.presetLaunchPower -= .1;
                telemetry.addData("Current Launcher Power (0-1): ",car.presetLaunchPower);
                telemetry.update();
                sleep(100);
            }

        }

    }
}
