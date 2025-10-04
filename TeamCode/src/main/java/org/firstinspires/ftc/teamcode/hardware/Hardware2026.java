package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Hardware2026 {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    public HardwareMap hwMap;

    //motors
    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;

    public DcMotorEx launcher = null;

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

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(1516.76, 1516.76, 950.833, 533.379)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCameraResolution(new Size(640,480));

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()



    public void createHardware() {

        initAprilTag();

        /*
        wheelFrontRight = hwMap.get(DcMotorEx.class,"rfWheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "lrWheel");

        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheelFrontRight.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackLeft.setVelocity(0);

        //init GoBuilda Odameter
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");

        //TODO: Modidfy these offsets to match 2026 Season robot
        odo.setOffsets(85, -125);
*/
        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        //odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
          Please refer to Go Builder Example.
         */
        //odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.resetPosAndIMU();

    }

    /**
     * Read the april tag
     * @return
     */
    //TODO: Modify this for use with Limelight Camera
    public int readGreenIndex () {
        //visionPortal.resumeStreaming();
        int greenIndex = 0;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            Log.d("9010", "Detection Id: " + detection.id );
            greenIndex = detection.id - 21;
        }
        //visionPortal.stopStreaming();
        return greenIndex;
    }

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

    /*
    public void intakeOn() {
        intake.setPower(1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void launcherOn() {
        launcher.setPower(1);
    }

    public void launcherOff() {
        launcher.setPower(0);
    }

     */


}
