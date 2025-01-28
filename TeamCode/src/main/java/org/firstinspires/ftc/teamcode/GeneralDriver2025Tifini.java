package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware2025Tifini;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2023;

@TeleOp(name="GeneralDriverTifini2025", group="TeleOps")
public class GeneralDriver2025Tifini extends LinearOpMode {

    private boolean debug = true;
    Hardware2025Tifini hdw;

    MecanumWheels2023 robotWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2025Tifini(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2023();

        double turbo = .6;

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        while (opModeIsActive()) {

            robotWheel.joystick(gamepad1, turbo);

            hdw.wheelFrontLeft.setVelocity(  robotWheel.wheelFrontLeftPower *  Hardware2025Tifini.ANGULAR_RATE );
            hdw.wheelBackLeft.setVelocity(robotWheel.wheelBackLeftPower *  Hardware2025Tifini.ANGULAR_RATE );
            hdw.wheelFrontRight.setVelocity(robotWheel.wheelFrontRightPower *  Hardware2025Tifini.ANGULAR_RATE );
            hdw.wheelBackRight.setVelocity(robotWheel.wheelBackRightPower *  Hardware2025Tifini.ANGULAR_RATE );


            if (gamepad1.left_bumper) {
                hdw.intakeWheelOn();
            }
            if (gamepad1.right_bumper) {
                hdw.intakeWheelOff();
            }
        }
    }

}
