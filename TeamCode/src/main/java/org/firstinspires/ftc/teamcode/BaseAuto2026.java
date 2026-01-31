package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.CarouelController;
import org.firstinspires.ftc.teamcode.hardware.Hardware2026;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2023;

abstract public class BaseAuto2026 extends LinearOpMode {
    int tagId ;
    int greenIndex = 0;

    Hardware2026 hdw;
    CarouelController car;

    abstract void moveToReadPosition ();

    abstract  void moveToShootingPosition() ;

    abstract  void movePark();

    abstract void moveToIntake();

    abstract  void moveAfterShoot();

    void intakeSequence() {
        try {
            hdw.setIntakePower(1);
            hdw.moveToXYPosition(140, 0, 0);
            car.rotateOneSlotCW();
            hdw.moveToXYPosition(140, 0, 0);
            car.rotateOneSlotCW();
            hdw.moveToXYPosition(140, 0, 0);
            hdw.setIntakePower(0);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

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
        moveToReadPosition();
        greenIndex = hdw.readGreenIndex();
        telemetry.addData("Green Index Read: ", greenIndex);
        telemetry.update();
        moveToShootingPosition();
        car.shootPattern(greenIndex);
        //movePark();
        //moveToIntake();
        //intakeSequence();
        //moveAfterShoot();
        //car.shootPattern(greenIndex);

    }

}
