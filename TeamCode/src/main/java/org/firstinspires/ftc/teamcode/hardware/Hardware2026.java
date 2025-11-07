package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Hardware2026 {

    private Limelight3A limelight;

    public HardwareMap hwMap;

    //motors
    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;

    private DcMotor launcher = null;
    private float launcherPower = 0;
    private DcMotor intake = null;
    private float intakePower = 0 ;

    private Servo lanchLever = null;
    private final float leverLowPosition =(float) 0.3;


    //declare GoBuilda Odometry
    GoBildaPinpointDriver odo;

    //TODO: Recalibrate these PID values for the next robot
    private double turnKP = 15;
    private double turnKI = 1;
    private double turnKD = 0.5;
    private double turnKF = 0.0;

    private double lnKP = 3.5;
    private double lnKI = 0.8;
    private double lnKD = 1.4;
    private double lnKF = 0.0;

    private double moveTimeOut = 7000;

    public Telemetry telemetry;

    static public double ANGULAR_RATE = 2000;

    public Hardware2026(HardwareMap m, Telemetry tm) {
        hwMap = m;
        telemetry = tm;
    }

    public float getIntakePower() {
        return intakePower;
    }

    public float getLauncherPower() {
        return launcherPower;
    }

    public void createHardware() {
        //Initialize LimeLite
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        wheelFrontRight = hwMap.get(DcMotorEx.class,"rfWheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "lrWheel");

        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelBackLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelBackRight.setDirection(DcMotor.Direction.REVERSE);

        wheelFrontRight.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackLeft.setVelocity(0);

        launcher = hwMap.get(DcMotorEx.class,"launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hwMap.get(DcMotorEx.class,"intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        lanchLever = hwMap.get(Servo.class,"launchLever");
        lanchLever.setPosition(leverLowPosition);

        //init GoBuilda Odameter
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");

        //X pod is 4 inch on the right of center,  Y pod is 2 inch behind the center.
        odo.setOffsets(-101, -52);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
          Please refer to Go Builder Example.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        odo.resetPosAndIMU();


    }

    /**
     * Read the april tag
     *
     * @return  return the index, 1 2 or 3.
     * If failed to read, it'll return 0.  In auto,  it shall loop till a timeout to find a valid
     * reading.
     */
    public int readGreenIndex () {

        int greenIndex = 0;
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                greenIndex = fr.getFiducialId() - 20;
            }
        }
        return greenIndex;
    }


    /**
     * Move Robot according to the position of the April Tag.  Robot suppose to be square with the
     * april tag
     *
     * @param tagId    Id of the tag to be used for reference.
     * @param targetY   distance of robot to the april tag,  unit in inches.
     * @param targetX   horizontal shift to the center of april tag.  unit in inches.  Positive
     *                  means tag is on the right of robot camera.
     */
    public void moveByAprilTag( int tagId,  double targetY  ,  double targetX  ) throws InterruptedException {
        //TODO: Recauculate this.
        //Yaw difference between camera and robot front line.
        double yawOffset = 0;
        //Center of robot to the camera,
        double cameraRadius = 6.875 ;

        Log.d("9010", "in MoveByApril Tag, Target Tag is: "  + tagId );

        //Start April Tag detection fo find tag, possible multiple tag in camera frame,
        //So result is a list.
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if ( fr.getFiducialId() == tagId) {
                        //Here we found our target April Tag.
                        //Get Initial Error
                        //double xToTag = fr.getTargetPoseCameraSpace().getPosition().x*1000;
                        //Log.d("9010", "xToTag: " + xToTag) ;

                        double yawToTag  = fr.getTargetPoseCameraSpace().getOrientation().getYaw();
                        double rangeToTag = fr.getTargetPoseCameraSpace().getPosition().z*1000;
                        Log.d("9010", " Yaw " + yawToTag + "range: " + rangeToTag);

                        double newX= rangeToTag * Math.sin( Math.toRadians( - yawToTag)  );
                        double newY = rangeToTag * Math.cos(Math.toRadians( - yawToTag));

                        //Move by odometer.
                        this.moveToXYPosition(newX - targetX , newY - targetY, yawToTag-yawOffset);

                    }
                }

        } else {
            telemetry.addData("No AprilTags Detected, exit ", tagId );
            telemetry.update();
            Log.d("9010", "No AprilTags Detected, Exit "  + tagId );
        }
    }


    /**
     * This operation moves robot to a position relative to its current position
     *
     * @param x       Target x position,  unit in mm   Positive to forward, negative to backward.
     * @param y       Target y position,  unix in mm,  Positive to left, negative to right.
     * @param heading Target heading, in degress,  Positive to turn left, negative to turn right.
     */
    public void moveToXYPosition(double x, double y, double heading) throws InterruptedException {
        Log.d("9010", "Entering into moveToXYPosition ");

        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odo.resetPosAndIMU();
        odo.bulkUpdate();
        while (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            Log.d("9010", "Status: " + odo.getDeviceStatus());
            Thread.sleep(10);
            odo.bulkUpdate();
        }

        odo.bulkUpdate();
        Pose2D initPos = odo.getPosition();
        Pose2D currentPos = null;

        //Before start, get init position and heading
        double currenXPosition = initPos.getX(DistanceUnit.MM);
        Log.d("9010", "current X Position " + currenXPosition);

        double currenYPosition = initPos.getY(DistanceUnit.MM);
        Log.d("9010", "current Y Position " + currenYPosition);

        double startHeading = initPos.getHeading(AngleUnit.DEGREES);
        Log.d("9010", "current Heading:  " + startHeading);

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
                && ((System.currentTimeMillis() - initMill) < moveTimeOut)) {
            //Get Odo meter reading
            odo.bulkUpdate();
            currentPos = odo.getPosition();
            /* Log.d ("9010", "odo readings,  X: "   +  currentPos.getX(DistanceUnit.MM)
                    + " Y: " + currentPos.getY(DistanceUnit.MM)
                    + " Heading: " + currentPos.getHeading(AngleUnit.DEGREES) );
            */
            //Reverse X and Y, Gobuilda PinPoint odo meter has X on Foward, and Y on Strafe
            double velocityXCaculated = lnYPidfCrtler.calculate(targetYPosition - currentPos.getY(DistanceUnit.MM));
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

    public void setIntakePower ( float power ) {
        intakePower=power;
        intake.setPower(power );
    }

    public void setLauncherPower (float power ) {
        launcherPower=power;
        launcher.setPower(power);
    }

    public void raiseLever ( ) {
        lanchLever.setPosition(1);
    }

    public void lowerLever() {
        lanchLever.setPosition(leverLowPosition);
    }

}
