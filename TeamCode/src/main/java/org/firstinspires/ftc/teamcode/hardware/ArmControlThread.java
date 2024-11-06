package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class runs on another thread to controll the arm motor position
 *
 */
public class ArmControlThread implements  Runnable {

    /**
     * Whether it shall run.
     */
    private boolean shallRun = true;

    private DcMotor elevation;

    private int eleInitPosition;
    private int position;

    ArmControlThread(DcMotor ele, int eleInitPosition) {
        this.elevation = ele;
        this.eleInitPosition = eleInitPosition;
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

        while ( this.shallRun == true ) {

            int targetPosition = eleInitPosition + position;

            //Move the slide
            int currentPosition = elevation.getCurrentPosition();
            //Log.d("9010", "elev position before Move: " + elevation.getCurrentPosition());

            int difference = targetPosition - currentPosition;
            Log.d("9010", "Difference:  " + difference );

            //Only set if difference is large otherwise do nothing.
            if ( Math.abs(difference) > 10 && ( targetPosition - eleInitPosition) > -50 ) {
                //if it's not busy, send new position command
                if (!elevation.isBusy()) {
                    elevation.setTargetPosition(targetPosition);
                    elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Log.d("9010", "Set Target position : " + targetPosition);
                    elevation.setPower(0.5);
                } else {

                }
            } else {
                elevation.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
                elevation.setPower(0);
            }

        }
    }
}
