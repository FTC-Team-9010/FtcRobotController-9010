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



    @Override
    public void runOpMode() {
        car = new CarouelController(hardwareMap);
        car.initialize();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                car.initPosition();
            } else if ( gamepad1.b) {
                car.rotateOneSlotCW();
            }
        }
    }
}
