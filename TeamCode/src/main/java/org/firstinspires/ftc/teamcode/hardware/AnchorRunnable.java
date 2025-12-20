package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AnchorRunnable implements  Runnable{
    Hardware2026 hdw;
    LinearOpMode opMode;

    private boolean running = false;

    public AnchorRunnable(Hardware2026 hdw, LinearOpMode opMode) {
        this.hdw = hdw;
        this.opMode = opMode;
        running = false;
    }

    public boolean isRunning() {
        return running;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    @Override
    public void run() {
        while (opMode.opModeIsActive()) {
            Log.d("9010","Into Thread");
            while (this.running) {
                hdw.wheelFrontLeft.setVelocity(0);
                hdw.wheelBackLeft.setVelocity(0);
                hdw.wheelFrontRight.setVelocity(0);
                hdw.wheelBackRight.setVelocity(0);

                hdw.odo.resetPosAndIMU();
                hdw.odo.bulkUpdate();
                while (hdw.odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    hdw.odo.bulkUpdate();
                }
                //Log.d("9010", "Status: " + hdw.odo.getDeviceStatus());

                Pose2D initPos = hdw.odo.getPosition();
                Pose2D currentPos = null;

                //Before start, get init position and heading
                double initPosX = initPos.getX(DistanceUnit.MM);

                double initPosY = initPos.getY(DistanceUnit.MM);

                double startHeading = initPos.getHeading(AngleUnit.DEGREES);

                //Initialize PID Controller
                PIDFController lnYPidfCrtler = new PIDFController(hdw.lnKP, hdw.lnKI, hdw.lnKD, hdw.lnKF);
                //Log.d("9010", "lnYKp: " + lnKP + "  lnKI: " + lnKI + " lnKD: " + lnKD);
                //Give X compansation more KP
                PIDFController lnXPidfCrtler = new PIDFController(hdw.lnKP, hdw.lnKI, hdw.lnKD, hdw.lnKF);
                //Log.d("9010", "lnXKp: " + lnKP + "  lnXKI: " + lnKI + " lnXKD: " + lnKD);
                PIDFController turnPidfCrtler = new PIDFController(hdw.turnKP, hdw.turnKI, hdw.turnKD, hdw.turnKF);
                //Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);

                lnYPidfCrtler.setSetPoint(0);
                lnYPidfCrtler.setTolerance(5);
                //set Integration to avoid saturating PID output.
                lnYPidfCrtler.setIntegrationBounds(-1000, 1000);

                lnXPidfCrtler.setSetPoint(0);
                lnXPidfCrtler.setTolerance(5);
                lnXPidfCrtler.setIntegrationBounds(-1000, 1000);

                turnPidfCrtler.setSetPoint(0);
                //Set tolerance as 0.5 degrees
                turnPidfCrtler.setTolerance(0.2);
                turnPidfCrtler.setIntegrationBounds(-1, 1);

                while (running) {
                    //Get Odo meter reading
                    hdw.odo.bulkUpdate();
                    while (hdw.odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                        try {
                            Thread.sleep(10);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        hdw.odo.bulkUpdate();
                    }
                    //Log.d("9010", "Status: " + odo.getDeviceStatus());
                    currentPos = hdw.odo.getPosition();
                    //Log.d ("9010", "odo readings,  X: "   +  currentPos.getX(DistanceUnit.MM)
                    //        + " Y: " + currentPos.getY(DistanceUnit.MM)
                    //        + " Heading: " + currentPos.getHeading(AngleUnit.DEGREES) );

                    //Reverse X and Y, Gobuilda PinPoint odo meter has X on Foward, and Y on Strafe
                    double velocityXCaculated = lnYPidfCrtler.calculate(initPosY - currentPos.getY(DistanceUnit.MM));
                    double velocityYCaculated = lnXPidfCrtler.calculate(initPosX - currentPos.getX(DistanceUnit.MM));
                    double rx = -turnPidfCrtler.calculate(startHeading - currentPos.getHeading(AngleUnit.DEGREES));

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

                    hdw.wheelFrontLeft.setVelocity(frontLeftVelocity);
                    hdw.wheelBackLeft.setVelocity(backLeftVelocity);
                    hdw.wheelFrontRight.setVelocity(frontRightVelocity);
                    hdw.wheelBackRight.setVelocity(backRightVelocity);
                }

                Log.d("9010", "odo readings after move,  X: " + currentPos.getX(DistanceUnit.MM)
                        + " Y: " + currentPos.getY(DistanceUnit.MM)
                        + " Heading: " + currentPos.getHeading(AngleUnit.DEGREES));

                Log.d("9010", "Before Stop");
                hdw.wheelFrontLeft.setVelocity(0);
                hdw.wheelBackLeft.setVelocity(0);
                hdw.wheelFrontRight.setVelocity(0);
                hdw.wheelBackRight.setVelocity(0);
                Log.d("9010", "After Stop wheels");

            }
        }
    }
}
