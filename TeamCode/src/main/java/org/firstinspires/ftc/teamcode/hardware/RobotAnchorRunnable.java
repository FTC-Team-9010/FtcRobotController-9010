package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotAnchorRunnable implements Runnable  {

    private Hardware2026 hdw = null;
    private LinearOpMode op;

    boolean active = true;

    public RobotAnchorRunnable ( Hardware2026 hardware, LinearOpMode opMode ) {
        hdw= hardware;
        op = opMode;
    }

    public void resetAnchor () {
        Log.d("9010", "Entering into AnchorRunble  ");

        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Put motor back into run with encoder mode.
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hdw.odo.resetPosAndIMU();
        hdw.odo.bulkUpdate();
        while (hdw.odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            Log.d("9010", "Status: " + hdw.odo.getDeviceStatus());
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            hdw.odo.bulkUpdate();
        }

        hdw.odo.bulkUpdate();
        Pose2D initPos = hdw.odo.getPosition();
        Pose2D currentPos = null;

        //Before start, get init position and heading
         currenXPosition = initPos.getX(DistanceUnit.MM);
        Log.d("9010", "current X Position " + currenXPosition);

         currenYPosition = initPos.getY(DistanceUnit.MM);
        Log.d("9010", "current Y Position " + currenYPosition);

         startHeading = initPos.getHeading(AngleUnit.DEGREES);
        Log.d("9010", "current Heading:  " + startHeading);

    }
    double currenXPosition;
    double currenYPosition;
    double startHeading;


    @Override
    public void run() {
        /*
        while (active && op.opModeIsActive() ) {

                double targetXPosition = currenXPosition + x;
                double targetYPosition = currenYPosition + y;
                double targetHeading = startHeading + heading;

                //Initialize PID Controller
                PIDFController lnYPidfCrtler = new PIDFController(lnKP, lnKI, lnKD, lnKF);
                Log.d("9010", "lnYKp: " + lnKP + "  lnKI: " + lnKI + " lnKD: " + lnKD);
                //Give X compansation more KP
                PIDFController lnXPidfCrtler = new PIDFController(lnKP, lnKI, lnKD, lnKF);
                Log.d("9010", "lnXKp: " + lnKP + "  lnXKI: " + lnKI + " lnXKD: " + lnKD);
                PIDFController turnPidfCrtler = new PIDFController(turnKP, turnKI, turnKD, turnKF);
                Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);

                lnYPidfCrtler.setSetPoint(0);
                lnYPidfCrtler.setTolerance(10);
                //set Integration to avoid saturating PID output.
                lnYPidfCrtler.setIntegrationBounds(-1000, 1000);

                lnXPidfCrtler.setSetPoint(0);
                lnXPidfCrtler.setTolerance(10);
                lnXPidfCrtler.setIntegrationBounds(-1000, 1000);

                turnPidfCrtler.setSetPoint(0);
                //Set tolerance as 0.5 degrees
                turnPidfCrtler.setTolerance(1);
                turnPidfCrtler.setIntegrationBounds(-1, 1);

                Log.d("9010", "Before entering Loop ");

                long initMill = System.currentTimeMillis();

                while (!(lnYPidfCrtler.atSetPoint() && lnXPidfCrtler.atSetPoint() && turnPidfCrtler.atSetPoint())
                        && ((System.currentTimeMillis() - initMill) < hdw.moveTimeOut)) {
                    //Get Odo meter reading
                    hdw.odo.bulkUpdate();
                    hdw.currentPos = hdw.odo.getPosition();
                    //Reverse X and Y, Gobuilda PinPoint odo meter has X on Foward, and Y on Strafe
                    double velocityYCaculated = lnXPidfCrtler.calculate(targetXPosition - currentPos.getX(DistanceUnit.MM));
                    double rx = -turnPidfCrtler.calculate(targetHeading - currentPos.getHeading(AngleUnit.DEGREES));

                    //Log.d("9010", "Error X: " + (targetXPosition - currentPos.getX(DistanceUnit.MM) ) );
                    //Log.d("9010", "Error Y: " + (targetYPosition - currentPos.getY(DistanceUnit.MM) ));
                    //Log.d("9010", "Error heading: " + (targetHeading - currentPos.getHeading(AngleUnit.DEGREES)) );

                    //Log.d("9010", "velocityYCaculated: " + velocityYCaculated ) ;
                    //Log.d("9010", "velocityXCaculated " + velocityXCaculated );
                    //Log.d("9010", "rx: "  + rx ) ;

                    //As GoBuilda PinPoint driver gives field centric reading for x and y,
                    // We need to use the field centric formula.

                    // Rotate the movement direction counter to the bot's rotation
                    double beta = -Math.toRadians(currentPos.getHeading(AngleUnit.DEGREES) - startHeading);
                    double rotX = velocityXCaculated * Math.cos(-beta) -
                            velocityYCaculated * Math.sin(-beta);
                    double rotY = velocityXCaculated * Math.sin(-beta)
                            + velocityYCaculated * Math.cos(-beta);

                    //Log.d("9010", "RotX: " + rotX ) ;
                    //Log.d("9010", "rotY " + rotY );

                    double frontLeftVelocity = (rotY - rotX + rx);
                    double backLeftVelocity = (rotY + rotX + rx);
                    double frontRightVelocity = (rotY + rotX - rx);
                    double backRightVelocity = (rotY - rotX - rx);

                    wheelFrontLeft.setVelocity(frontLeftVelocity);
                    wheelBackLeft.setVelocity(backLeftVelocity);
                    wheelFrontRight.setVelocity(frontRightVelocity);
                    wheelBackRight.setVelocity(backRightVelocity);
                }
            }
*/

    }


}
