package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.CarouelController;
import org.firstinspires.ftc.teamcode.hardware.Hardware2026;

@Autonomous(name = "ParkAuto")
public class ParkAuto extends LinearOpMode {

    Hardware2026 hdw;
    CarouelController car;

    @Override
    public void runOpMode() throws InterruptedException {
        car = new CarouelController(hardwareMap, telemetry);
        car.initialize();

        hdw = new Hardware2026(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        telemetry.addData("[>]", "All set?");
        telemetry.update();
        //car.initPosition();

        waitForStart();
        telemetry.clearAll();
        hdw.moveToXYPosition(800,0,0);

    }
}
