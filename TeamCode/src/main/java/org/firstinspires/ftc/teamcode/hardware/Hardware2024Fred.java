package org.firstinspires.ftc.teamcode.hardware;



import com.arcrobotics.ftclib.controller.PIDFController;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;

import java.util.Locale;

public class Hardware2024Fred {

    public HardwareMap hwMap;

    //motors
    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;
    public DcMotorEx vSlide = null;
    public DcMotorEx elevation = null;

    public Servo claw = null;

    // Declare OpMode member for the Odometry Computer
    GoBildaPinpointDriver odo;

    //This is max wheel and slide motor velocity.
    static public double ANGULAR_RATE = 2500.0;

    private final double xAxisCoeff = 216.5;  // How many degrees encoder to turn to run an inch in X Axis
    private final double yAxisCoeff = 216.5;  // How many degrees encoder to turn to run an inch in Y Axis

    private boolean debug = true;
    private Telemetry telemetry;

    private final  double slideUpperLimit = 4000;
    //2750 is about 1/4 inch longer during inspection on 1st league meet  11/2/2024
    private final double  slideHorizontalLimit = 2700;
    public final double elevLimit = 1000;
    //Only allow slide to extend longer after passthing this threashold.
    private final double elevThreadshold = 900;

    //PID control parameter for turning & linear movement.
    private double turnKP = 15;
    private double turnKI = 1;
    private double turnKD = 0.5;
    private double turnKF = 0.0;

    private double lnKP = 15;
    private double lnKI = 1;
    private double lnKD = 0.5;
    private double lnKF = 0.0;

    private double moveTimeOut = 5000;

    public double getLnKF() {
        return lnKF;
    }

    public double getTurnKP() {
        return turnKP;
    }

    public void setTurnKP(double turnKP) {
        this.turnKP = turnKP;
    }

    public double getTurnKI() {
        return turnKI;
    }

    public void setTurnKI(double turnKI) {
        this.turnKI = turnKI;
    }

    public double getTurnKD() {
        return turnKD;
    }

    public void setTurnKD(double turnKD) {
        this.turnKD = turnKD;
    }

    public double getTurnKF() {
        return turnKF;
    }

    public void setTurnKF(double turnKF) {
        this.turnKF = turnKF;
    }

    public void setLnKF(double lnKF) {
        this.lnKF = lnKF;
    }

    public double getLnKD() {
        return lnKD;
    }

    public void setLnKD(double lnKD) {
        this.lnKD = lnKD;
    }

    public double getLnKI() {
        return lnKI;
    }

    public void setLnKI(double lnKI) {
        this.lnKI = lnKI;
    }

    public double getLnKP() {
        return lnKP;
    }

    public void setLnKP(double lnKP) {
        this.lnKP = lnKP;
    }

    /**
     * Constructor
     *
     * @param m  This is the HardwareMap, which is configured on the driver station.
     * @param tm The Telemetry object, used for debug purpose.
     */
    public Hardware2024Fred(HardwareMap m, Telemetry tm) {
        hwMap = m;
        telemetry = tm;
        odo = hwMap.get(GoBildaPinpointDriver.class,"odo");

    }


    private int vsldieInitPosition = 0;
    private int elevInitPosition = 0;

    /**
     * Initialize hardware.
     */
    public void createHardware() {
        //Wheels
        wheelFrontRight = hwMap.get(DcMotorEx.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "lrWheel");

        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelBackLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelBackRight.setDirection(DcMotor.Direction.REVERSE);

        wheelFrontRight.setPower(0);
        wheelBackRight.setPower(0);
        wheelBackLeft.setPower(0);
        wheelBackRight.setPower(0);

        claw = hwMap.get(Servo.class, "claw");

        vSlide = hwMap.get(DcMotorEx.class, "vSlide");
        elevation = hwMap.get(DcMotorEx.class, "elev");
        vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        vsldieInitPosition = vSlide.getCurrentPosition() ;
        elevInitPosition = elevation.getCurrentPosition() ;
        elevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //init GoBuilda Odameter
        odo = hwMap.get(GoBildaPinpointDriver.class,"odo");

        /*
          Please refer to Go Builder Example.
         */
        odo.setOffsets(30.0, 140.0); //these are tuned for 3110-0002-0001 Product Insight #1

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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

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
     * Regulate degreee between -180 and 180, by adding or subtracting 360.
     *
     * @param degree
     * @return
     */
    private double regulateDegree(double degree) {
        if (degree > 180) {
            degree -= 360;
        } else if (degree < -180) {
            degree += 360;
        }

        return degree;
    }



    public void freeMoveSlide( float power ) {
        double slidePosition  = vSlide.getCurrentPosition();
        Log.d("9010", "vSlide position " + slidePosition);
        double elePosition = elevation.getCurrentPosition();
        Log.d("9010", "ele position " + slidePosition);

        //Control  Vslide
        if ( (power > 0 && ( ( elePosition < elevThreadshold &&  slidePosition < slideHorizontalLimit ) ||
                             ( elePosition > elevThreadshold &&  slidePosition < slideUpperLimit) ) )
                || (power < 0 && slidePosition > 0)) {
            vSlide.setVelocity(power * ANGULAR_RATE);
        } else {
            vSlide.setVelocity(0);
        }

    }
    public void freeMoveElevation(float power) {
        double elePosition   = elevation.getCurrentPosition();
        //Log.d("9010", "ele position " + elePosition);

        if ((power > 0 && elePosition < elevLimit) || (power < 0 && elePosition > 0)) {
            elevation.setVelocity(power * ANGULAR_RATE);
        } else {
            elevation.setVelocity(0);
        }
    }

    public void goElevation ( int  position  ) {

        int targetPosition = elevInitPosition + position;

        //Move the slide
        int currentPosition = elevation.getCurrentPosition();
        //Log.d("9010", "elev position before Move: " + elevation.getCurrentPosition());

        int difference = targetPosition - currentPosition;
        Log.d("9010", "Difference:  " + difference );

        //Only set if difference is large otherwise do nothing.
        if ( Math.abs(difference) > 10 && ( targetPosition - elevInitPosition) > -50 ) {
            //if it's not busy, send new position command
            if (!elevation.isBusy()) {
                elevation.setTargetPosition(targetPosition);
                elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Log.d("9010", "Set Target position : " + targetPosition);
                elevation.setPower(0.5);
            } else {

            }
        } else {
            elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevation.setPower(0);
            elevation.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //Log.d("9010", "Inside Moving Loop : " + vSlide.getCurrentPosition() + " Sign: " + sign);

        //Log.d("9010", "Elev after Move : " + elevation.getCurrentPosition());

    }



    /**
     * This operation moves robot to a position relative to its current position
     * @param x  Target x position,  unit in mm   Positive to forward, negative to backward.
     * @param y  Target y position,  unix in mm,  Positive to left, negative to right.
     * @param heading Target heading, in degress,  Positive to turn left, negative to turn right.
     */
    public  void moveToXYPosition(double x , double y, double heading ) throws InterruptedException {
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
        while ( odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
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
        double targetYPosition =currenYPosition + y;
        double targetHeading = startHeading + heading;

        //Initialize PID Controller
        PIDFController lnYPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnYKp: " + lnKP + "  lnKI: " + lnKI + " lnKD: " + lnKD);
        //Give X compansation more KP
        PIDFController lnXPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnXKp: " + lnKP + "  lnXKI: " + lnKI + " lnXKD: " + lnKD);
        PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);

        lnYPidfCrtler.setSetPoint(0);
        lnYPidfCrtler.setTolerance(10);
        //set Integration to avoid saturating PID output.
        lnYPidfCrtler.setIntegrationBounds(-1000 , 1000);

        lnXPidfCrtler.setSetPoint(0);
        lnXPidfCrtler.setTolerance(10);
        lnXPidfCrtler.setIntegrationBounds(-1000 , 1000);

        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(1);
        turnPidfCrtler.setIntegrationBounds(-1 , 1 );

        Log.d("9010", "Before entering Loop ");

        long initMill = System.currentTimeMillis();

        while ( !(lnYPidfCrtler.atSetPoint()&&lnXPidfCrtler.atSetPoint() && turnPidfCrtler.atSetPoint() )
                && ( (System.currentTimeMillis() -initMill  )< moveTimeOut)  ) {
            //Get Odo meter reading
            odo.bulkUpdate();
            currentPos  = odo.getPosition();
            Log.d ("9010", "odo readings,  X: "   +  currentPos.getX(DistanceUnit.MM)
                    + " Y: " + currentPos.getY(DistanceUnit.MM)
                    + " Heading: " + currentPos.getHeading(AngleUnit.DEGREES) );

            //Reverse X and Y, Gobuilda PinPoint odo meter has X on Foward, and Y on Strafe
            double velocityXCaculated = lnYPidfCrtler.calculate(targetYPosition -currentPos.getY(DistanceUnit.MM) ) ;
            double velocityYCaculated = lnXPidfCrtler.calculate(targetXPosition - currentPos.getX(DistanceUnit.MM) );
            double rx = -turnPidfCrtler.calculate(  targetHeading - currentPos.getHeading(AngleUnit.DEGREES) );

            Log.d("9010", "Error X: " + (targetXPosition - currentPos.getX(DistanceUnit.MM) ) );
            Log.d("9010", "Error Y: " + (targetYPosition - currentPos.getY(DistanceUnit.MM) ));
            Log.d("9010", "Error heading: " + (targetHeading - currentPos.getHeading(AngleUnit.DEGREES)) );

            Log.d("9010", "velocityYCaculated: " + velocityYCaculated ) ;
            Log.d("9010", "velocityXCaculated " + velocityXCaculated );
            Log.d("9010", "rx: "  + rx ) ;

            //As GoBuilda PinPoint driver gives field centric reading for x and y,
            // We need to use the field centric formula.

            // Rotate the movement direction counter to the bot's rotation
            double beta = -Math.toRadians (currentPos.getHeading(AngleUnit.DEGREES ) - startHeading) ;
            double rotX = velocityXCaculated * Math.cos(-beta) -
                    velocityYCaculated * Math.sin(-beta);
            double rotY = velocityXCaculated * Math.sin(-beta)
                    + velocityYCaculated * Math.cos(-beta);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing

            Log.d("9010", "RotX: " + rotX ) ;
            Log.d("9010", "rotY " + rotY );

            //double denominator = Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx);
            double frontLeftVelocity = (rotY - rotX + rx) ;
            double backLeftVelocity = (rotY + rotX + rx) ;
            double frontRightVelocity = (rotY + rotX - rx) ;
            double backRightVelocity  = (rotY - rotX - rx) ;

            wheelFrontLeft.setVelocity(frontLeftVelocity);
            wheelBackLeft.setVelocity(backLeftVelocity);
            wheelFrontRight.setVelocity(frontRightVelocity);
            wheelBackRight.setVelocity(backRightVelocity);
        }

        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);

    }

    public void openClaw() {
        claw.setPosition(0);
    }

    public void closeClaw() {
        claw.setPosition(1);
    }



}