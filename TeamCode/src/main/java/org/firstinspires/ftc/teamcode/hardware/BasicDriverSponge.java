package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.teamcode.hardware.HardwareSponge;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2022;


@TeleOp(name="GeneralDriverSponge", group="TeleOps")
public class BasicDriverSponge extends LinearOpMode{


    private boolean debug = true;
    HardwareSponge hdw;

    MecanumWheels2022 robotWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new HardwareSponge(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2022();

        double powerDrivePercentage = 0.80;

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        //This is the main loop of operation.
        while (opModeIsActive()) {
            //hdw.checkAndGrabCone();

            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%
            robotWheel.joystick(gamepad1, 1);

            /* Set the calcuated velocity to wheels according to the gampad input */
            double frontLeftVelocity = robotWheel.wheelFrontLeftPower * powerDrivePercentage;
            double backLeftVelocity = robotWheel.wheelBackLeftPower * powerDrivePercentage;
            double frontRightVelocity = robotWheel.wheelFrontRightPower * powerDrivePercentage;
            double backRightVelocity = robotWheel.wheelBackRightPower * powerDrivePercentage;
            telemetry.addData("here", "got here");
            telemetry.update();

            hdw.wheelFrontLeft.setPower(frontLeftVelocity);
            hdw.wheelBackLeft.setPower(backLeftVelocity);
            hdw.wheelFrontRight.setPower(frontRightVelocity);
            hdw.wheelBackRight.setPower(backRightVelocity);


        }

    }
}