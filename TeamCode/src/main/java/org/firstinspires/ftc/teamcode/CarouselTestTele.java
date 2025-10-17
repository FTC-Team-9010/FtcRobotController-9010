package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CarouelController;
import org.firstinspires.ftc.teamcode.hardware.Hardware2026;

@TeleOp (name="CarouselTestOp", group="TeleOps")
public class CarouselTestTele extends LinearOpMode {
    Hardware2026 hdw;
    CarouelController car;

    double[] pidCoffs = { 7,0.8,0.002 };
    int pidCoffIndex = 0;

    Hardware2026 hardware;

    @Override
    public void runOpMode() {
        car = new CarouelController(hardwareMap, telemetry);
        car.initialize();
        hardware = new Hardware2026(hardwareMap,telemetry);
        hardware.createHardware();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.back) {
                car.initPosition();
            } else if ( gamepad1.dpad_left) {
                car.setTurnKP(pidCoffs[0]);
                car.setTurnKI(pidCoffs[1]);
                car.setTurnKD(pidCoffs[2]);
                car.rotateOneSlotCW();
            } else if (gamepad1.dpad_right){
                car.setTurnKP(pidCoffs[0]);
                car.setTurnKI(pidCoffs[1]);
                car.setTurnKD(pidCoffs[2]);
                car.rotateOneSlotCCW();
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
                if (pidCoffIndex == 2 ) {
                    pidCoffs[pidCoffIndex] -= .001;
                } else {
                    pidCoffs[pidCoffIndex] -= .1;
                }
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
                sleep(100);
            }

            if( gamepad1.right_bumper) {
                if (pidCoffIndex == 2 ) {
                    pidCoffs[pidCoffIndex] += .001;
                } else {
                    pidCoffs[pidCoffIndex] += .1;
                }
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
                sleep(100);
            }

            if (gamepad1.dpad_up) {
                //int greenIndex = hardware.readGreenIndex();
                car.readBallConfiguration();
                car.matchConfigToSequence( 1);

            }

            if (gamepad1.dpad_down) {
                car.shootGreen();
            }

            if (gamepad1.left_stick_button) {
                car.shootPurple();

            }

            if (gamepad1.right_stick_button) {
                car.shootPattern(1);
            }

        }
    }
}
