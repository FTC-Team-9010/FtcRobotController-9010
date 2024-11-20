package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware2022;
import org.firstinspires.ftc.teamcode.hardware.Hardware2024Fred;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2023;

@TeleOp(name="GeneralDriverFred2024", group="TeleOps")

public class GeneralDriverFred2024 extends LinearOpMode {

    private boolean debug = true;
    Hardware2024Fred hdw;

    MecanumWheels2023 robotWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2024Fred(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2023();

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        int elevationPosition = 0;
        double turbo = 1;

        //This is the main loop of operation.
        while (opModeIsActive()) {
            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%

            robotWheel.joystick(gamepad1, turbo);

            hdw.wheelFrontLeft.setVelocity(  robotWheel.wheelFrontLeftPower *  Hardware2024Fred.ANGULAR_RATE );
            hdw.wheelBackLeft.setVelocity(robotWheel.wheelBackLeftPower *  Hardware2024Fred.ANGULAR_RATE );
            hdw.wheelFrontRight.setVelocity(robotWheel.wheelFrontRightPower *  Hardware2024Fred.ANGULAR_RATE );
            hdw.wheelBackRight.setVelocity(robotWheel.wheelBackRightPower *  Hardware2024Fred.ANGULAR_RATE );

            hdw.freeMoveSlide(gamepad1.right_trigger - gamepad1.left_trigger);

            if (gamepad1.right_bumper) {
                if ( elevationPosition< hdw.elevLimit) {
                    elevationPosition += 5;
                }
                telemetry.clearAll();
                telemetry.addData("[>]", "Elevation Position: " + elevationPosition);
                telemetry.update();
            }

            if (gamepad1.left_bumper) {
                //make it faster move down
                if ( elevationPosition > 0 ) {
                    elevationPosition -= 5;
                }
                telemetry.clearAll();
                telemetry.addData("[>]", "Elevation Position: " + elevationPosition);
                telemetry.update();
            }

            //Set the arm to elevation for pick up speciman on the wall
            if (gamepad1.dpad_up) {
                elevationPosition = 570;
                //hdw.moveSlide(0);
            }

            hdw.goElevation( elevationPosition);

            if (gamepad1.x) {
                hdw.closeClaw();
            }

            if (gamepad1.y) {
                hdw.openClaw();

            }

            if (gamepad1.a) {
                turbo = 1;
                telemetry.clearAll();
                telemetry.addData("[>]", "Turbo:  " + turbo);
                telemetry.update();

            }

            if (gamepad1.b) {
                turbo = 0.1 ;
                telemetry.clearAll();
                telemetry.addData("[>]", "Turbo:  " + turbo);
                telemetry.update();
            }

        }

    }

}

