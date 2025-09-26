package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.ArmControlRunable;
import org.firstinspires.ftc.teamcode.hardware.CarouelController;
import org.firstinspires.ftc.teamcode.hardware.Hardware2024Fred;
import org.firstinspires.ftc.teamcode.hardware.Hardware2025Tifini;


@TeleOp(name="HDWTestOp", group="TeleOps")
public class HWTestTele  extends LinearOpMode {
    Hardware2025Tifini hdw;

    double[] pidCoffs = { 3.5,0.8,1.4 };
    int pidCoffIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        hdw = new Hardware2025Tifini(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        /*
        Log.d("9010", "Before create thread!");
        ArmControlRunable armRunable = new ArmControlRunable( hdw.getElevation(),
                hdw.getElevInitPosition( ), this) ;
        Thread armThread = new Thread(armRunable);
        armThread.start();
        Log.d("9010", "new thread running");

        armRunable.setPosition(300);

         */
        waitForStart();
        telemetry.clearAll();

        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();


        //This is the main loop of operation.
        while (opModeIsActive()) {
            //Record previous Gamepad Status
            previousGamePad1.copy(currentGamePad1);
            //Update current gamepad status
            currentGamePad1.copy(gamepad1);

            if (gamepad1.dpad_left) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveToXYPosition(0, 1200, 0 );
            }
            if (gamepad1.dpad_right) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveToXYPosition(0,  -1200 ,  0);
            }
            if (gamepad1.dpad_up) {
                telemetry.addLine().addData("[moving y >]  ", " Y ");
                telemetry.update();
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveToXYPosition(450, 0 , 0 );
            }
            if (gamepad1.dpad_down) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveToXYPosition (-450, 0, 0 );
            }


            if( gamepad1.x) {
                pidCoffIndex = 0;
                telemetry.addLine().addData("[*Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
            }

            if( gamepad1.y) {
                pidCoffIndex = 1;
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[*Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
            }

            if( gamepad1.a) {
                pidCoffIndex = 2;
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[*Kd :]  ", pidCoffs[2]);
                telemetry.update();
            }

            if( gamepad1.left_bumper) {
                    pidCoffs[pidCoffIndex] -= .1;

                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
                sleep(100);
            }

            if( gamepad1.right_bumper) {
                    pidCoffs[pidCoffIndex] += .1;

                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
                sleep(100);
            }

        }
    }

}
