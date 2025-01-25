package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware2025Tifini;
import org.firstinspires.ftc.teamcode.hardware.HardwareSponge;

@TeleOp(name="GeneralDriverTifini2025", group="TeleOps")
public class GeneralDriver2025Tifini extends LinearOpMode {

    private boolean debug = true;
    Hardware2025Tifini hdw;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2025Tifini(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                hdw.intakeWheelOn();
            }
            if (gamepad1.right_bumper) {
                hdw.intakeWheelOff();
            }
        }
    }

}
