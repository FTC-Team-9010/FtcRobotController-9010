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

        //This is the main loop of operation.
        while (opModeIsActive()) {
            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%
            robotWheel.joystick(gamepad1, .5);

            hdw.wheelFrontLeft.setVelocity(  robotWheel.wheelFrontLeftPower *  Hardware2024Fred.ANGULAR_RATE );
            hdw.wheelBackLeft.setVelocity(robotWheel.wheelBackLeftPower *  Hardware2024Fred.ANGULAR_RATE );
            hdw.wheelFrontRight.setVelocity(robotWheel.wheelFrontRightPower *  Hardware2024Fred.ANGULAR_RATE );
            hdw.wheelBackRight.setVelocity(robotWheel.wheelBackRightPower *  Hardware2024Fred.ANGULAR_RATE );

            //hdw.freeMoveVerticalSlide(gamepad1.right_trigger - gamepad1.left_trigger);

            //hdw.elevation(gamepad2.right_trigger - gamepad2.left_trigger);
        }

    }

}

