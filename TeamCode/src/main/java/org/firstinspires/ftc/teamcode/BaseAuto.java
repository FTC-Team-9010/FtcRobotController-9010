package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware2024Fred;
import org.firstinspires.ftc.teamcode.hardware.TeamPropPosition;

//435 max ticks per second is 383.6

//@Autonomous(name = "BaseAuto")
public abstract class BaseAuto extends LinearOpMode {

    Hardware2024Fred hdw;
    abstract void park ();

    TeamPropPosition detectedPosition = null;

    @Override
    public void runOpMode() {
        hdw = new Hardware2024Fred(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        waitForStart();

        scoreSpiecemen ();
        //park();
        idle();
    }

    /**
     * Score the speicement
     */
    void scoreSpiecemen () {
        //move to position
        try {
            hdw.moveToXYPosition(400,0,0);
            hdw.moveSlide(300);
            hdw.goElevation(900);

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}
