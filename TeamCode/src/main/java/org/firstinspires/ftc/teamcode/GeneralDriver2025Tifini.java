package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

        Gamepad currentGamePad1 = new Gamepad();
        Gamepad previousGamePad1 = new Gamepad();

        currentGamePad1.copy(gamepad1);
        while (opModeIsActive()) {

            //Record previous Gamepad Status
            previousGamePad1.copy(currentGamePad1);
            //Update current gamepad status
            currentGamePad1.copy(gamepad1);


            robotWheel.joystick(gamepad1, turbo);

            hdw.wheelFrontLeft.setVelocity(  robotWheel.wheelFrontLeftPower *  Hardware2025Tifini.ANGULAR_RATE );
            hdw.wheelBackLeft.setVelocity(robotWheel.wheelBackLeftPower *  Hardware2025Tifini.ANGULAR_RATE );
            hdw.wheelFrontRight.setVelocity(robotWheel.wheelFrontRightPower *  Hardware2025Tifini.ANGULAR_RATE );
            hdw.wheelBackRight.setVelocity(robotWheel.wheelBackRightPower *  Hardware2025Tifini.ANGULAR_RATE );

            if (!previousGamePad1.x && currentGamePad1.x) {
                if ( !hdw.intakeWheelOff() ) {
                    hdw.intakeWheelOn();
                } else {
                    hdw.intakeWheelOff();
                }



            }

            if (currentGamePad1.a) {
                hdw.hSlideExtend();
            }

            if (currentGamePad1.b) {
                hdw.hSlideRetract();
            }

            hdw.freeMoveVerticalSlide(gamepad1.right_trigger - gamepad1.left_trigger);


        }
    }

}
