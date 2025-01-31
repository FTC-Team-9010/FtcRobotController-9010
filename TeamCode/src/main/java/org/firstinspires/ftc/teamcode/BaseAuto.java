package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ArmControlRunable;
import org.firstinspires.ftc.teamcode.hardware.Hardware2024Fred;
import org.firstinspires.ftc.teamcode.hardware.TeamPropPosition;

/**
 * This is the base class for autonomous.
 * There are some abstract method needs to be implemented by child class
 * based on starting position on the left or on the right.
 *
 */
public abstract class BaseAuto extends LinearOpMode {

    Hardware2024Fred hdw;

    /**
     * To park, left and right need to implement this.
     */
    abstract void park ();

    /**
     * Move to the position before score Specimen, this
     * needs to be implemented by left side or right side.
     *
     */
    abstract void moveBeforeSpecimen ();

    TeamPropPosition detectedPosition = null;
    ArmControlRunable armRunable;

    @Override
    public void runOpMode() {
        hdw = new Hardware2024Fred(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        hdw.closeClaw();

        waitForStart();
        //Create a thread to control the arm position.
        Log.d("9010", "Before create thread!");
        armRunable = new ArmControlRunable( hdw.getElevation(), hdw.getElevInitPosition());
        Thread armThread = new Thread(armRunable);
        armThread.start();
        Log.d("9010", "new thread running");

        raiseArm();
        moveBeforeSpecimen();
        scoreSpecimen ();
        park();

        //Stop the arm thread.
        armRunable.setShallRun(false);
        Log.d("9010", "Now put thread run = false ");
        idle();
    }

    /**
     * Raise the arm, it's used before move.
     */
    void raiseArm () {

         try {
             //Raiase the arm before move.
             armRunable.setPosition(300);
             //Because it's multithread, give it some time to raise the arm.
             Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Score the speicemen.
     * Before this method, robot is in the position before scoring the specimen.
     *
     */
    void scoreSpecimen ( ) {
        //
        try {
            Log.d("9010", "Into scoreSpicement, before movment.");
            hdw.moveToXYPosition(450,0,0);
            //raise arm to position for scoring.
            armRunable.setPosition(840);
            Thread.sleep(500);
            //Extend slide
            hdw.moveSlide(1600);
            //Move forward
            hdw.moveToXYPosition(70,0,0);
            //Retract slide a bit
            hdw.moveSlide(880);
            //Open claw
            hdw.openClaw();
            //Retract Slide all the way
            hdw.moveSlide(0);
            slowLowerArm();
            //Move back 24 in
            hdw.moveToXYPosition(-530, 0, 0 );

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

     }

    void slowLowerArm () throws InterruptedException {
        int armPosition = hdw.getElevation().getCurrentPosition();
        for (int p = armPosition; p > 0 ; p-=5 ) {
            armRunable.setPosition(p);
            Log.d("9010", "Set arm position: " + p);
            Thread.sleep(50);
        }
    }

}
