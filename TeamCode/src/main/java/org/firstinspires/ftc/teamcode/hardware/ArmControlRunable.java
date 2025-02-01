package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class runs on another thread to controll the arm motor position
 */
public class ArmControlRunable implements Runnable {

    /**
     * Whether it shall run.
     */
    private boolean shallRun = true;

    private DcMotor elevation;

    private int eleInitPosition;
    private int position;
    private LinearOpMode opMode ;

    public ArmControlRunable(DcMotor ele, int eleInitPosition, LinearOpMode baseAutoMode) {
        this.elevation = ele;
        this.eleInitPosition = eleInitPosition;
        this.position = 0;
        this.opMode = baseAutoMode;
    }

    public boolean isShallRun() {
        return shallRun;
    }

    public void setShallRun(boolean shallRun) {
        this.shallRun = shallRun;
    }

    public DcMotor getElevation() {
        return elevation;
    }

    public void setElevation(DcMotor elevation) {
        this.elevation = elevation;
    }

    public int getEleInitPosition() {
        return eleInitPosition;
    }

    public void setEleInitPosition(int eleInitPosition) {
        this.eleInitPosition = eleInitPosition;
    }

    public int getPosition() {
        return position;
    }

    public void setPosition(int position) {
        this.position = position;
    }

    @Override
    public void run() {
        Log.d("9010", "Into armThread");

        while (this.shallRun == true && this.opMode.opModeIsActive()) {

            int targetPosition = eleInitPosition + position;
            //Move the slide
            int currentPosition = elevation.getCurrentPosition();
            //Log.d("9010", "elev position before Move: " + elevation.getCurrentPosition());

            int difference = targetPosition - currentPosition;
            //Log.d("9010", "Difference:  " + difference + " target position: " + targetPosition
            //        + "eleInitPosition: " + eleInitPosition);

            //if it's not busy, send new position command
            if (!elevation.isBusy()) {
                //Only set if difference is large otherwise do nothing.
                elevation.setTargetPosition(targetPosition);
                elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //Log.d("9010", "Set Target position : " + targetPosition);
                elevation.setPower(0.7);
            } else {
                //Log.d("9010", "Motor busy!");
            }

        }
        Log.d("9010", "Thread run ends");
    }
}
