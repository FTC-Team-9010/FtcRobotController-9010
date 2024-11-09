package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ArmControlRunable;
import org.firstinspires.ftc.teamcode.hardware.Hardware2024Fred;
import org.firstinspires.ftc.teamcode.hardware.TeamPropPosition;

//435 max ticks per second is 383.6

//@Autonomous(name = "BaseAuto")
public abstract class BaseAuto extends LinearOpMode {

    Hardware2024Fred hdw;
    abstract void park ();

    TeamPropPosition detectedPosition = null;
    ArmControlRunable armRunable;

    @Override
    public void runOpMode() {
        hdw = new Hardware2024Fred(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        waitForStart();
        Log.d("9010", "Before create thread!");
        armRunable = new ArmControlRunable( hdw.getElevation(), hdw.getElevInitPosition());
        Thread armThread = new Thread(armRunable);
        armThread.start();

        Log.d("9010", "new thread running");

        scoreSpiecemen ();
        //park();
        armRunable.setShallRun(false);
        Log.d("9010", "Now put thread run = false ");
        idle();
    }

    /**
     * Score the speicement
     */
    void scoreSpiecemen () {
        //move to position
        try {
            Log.d("9010", "Into scoreSpicement, before movment.");
            hdw.moveToXYPosition(400,0,0);
            armRunable.setPosition(500);
            Log.d("9010", "Now sleep 5 seconds");
            Thread.sleep(5000);
            slowLowerArm();

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    void slowLowerArm () throws InterruptedException {
        int armPosition = hdw.getElevation().getCurrentPosition();
        for (int p = armPosition; p > 0 ; p-=10 ) {
            armRunable.setPosition(p);
            Thread.sleep(50);
        }
    }

}
