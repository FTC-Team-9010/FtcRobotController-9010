package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CarouelController;
import org.firstinspires.ftc.teamcode.hardware.Hardware2026;


@TeleOp(name="HDWTestOp", group="TeleOps")
public class HWTestTele  extends LinearOpMode {
    Hardware2026 hdw;

    double[] pidCoffs = { 15,0.3,3 };
    int pidCoffIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        hdw = new Hardware2026(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        telemetry.addData("[>]", "All set?");
        telemetry.update();

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
                hdw.setTurnKP(pidCoffs[0]);
                hdw.setTurnKI(pidCoffs[1]);
                hdw.setTurnKD(pidCoffs[2]);
                hdw.moveToXYPosition(0, 0, 10 );
            }
            if (gamepad1.dpad_right) {
                hdw.setTurnKP(pidCoffs[0]);
                hdw.setTurnKI(pidCoffs[1]);
                hdw.setTurnKD(pidCoffs[2]);
                hdw.moveToXYPosition(0,  0 ,  -90);
            }
            if (gamepad1.dpad_up) {
                telemetry.addLine().addData("[moving y >]  ", " Y ");
                telemetry.update();
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                //hdw.moveByAprilTag(20,1500,0);
                hdw.moveByAprilTag(20,2500);
            }

            if (gamepad1.dpad_down) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveByAprilTag(20, 3000);
                //hdw.moveToXYPosition(-1000,  0 ,  0);
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
