package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Hardware2025Tifini {

    //TODO: Recalculate these with new wheels
    private final double xAxisCoeff = 216.5;  // How many degrees encoder to turn to run an inch in X Axis
    private final double yAxisCoeff = 216.5;  // How many degrees encoder to turn to run an inch in Y Axis

    static public double ANGULAR_RATE = 2000;

    private boolean debug = true;
    private Telemetry telemetry;

    // Declare OpMode member for the Odometry Computer
    GoBildaPinpointDriver odo;

    private double turnKP = 15;
    private double turnKI = 1;
    private double turnKD = 0.5;
    private double turnKF = 0.0;

    private double lnKP = 3.5;
    private double lnKI = 0.8;
    private double lnKD = 1.4;
    private double lnKF = 0.0;

    private double moveTimeOut = 7000;

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

    private int vSlideInitPosition = 0;

    public Hardware2025Tifini(HardwareMap m, Telemetry tm) {
        hwMap = m;
        telemetry = tm;
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");

    }

    public HardwareMap hwMap;

    //motors
    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;

    public DcMotorEx vSlideRight = null;
    public DcMotorEx vSlideLeft = null;
    public DcMotorEx hSlide = null;

    //servos
    public CRServo intakeWheel = null;


    public void createHardware() {

        wheelFrontRight = hwMap.get(DcMotorEx.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "lrWheel");

        vSlideRight = hwMap.get(DcMotorEx.class, "vSlideR");
        vSlideLeft = hwMap.get(DcMotorEx.class, "vSlideL");
        hSlide = hwMap.get(DcMotorEx.class, "hSlide");

        intakeWheel = hwMap.get(CRServo.class, "intakeWheel");

        //TODO: Modify these later
        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelBackLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelBackRight.setDirection(DcMotor.Direction.REVERSE);

        vSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        vSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        hSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        wheelFrontRight.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackLeft.setVelocity(0);

        vSlideRight.setVelocity(0);
        vSlideLeft.setVelocity(0);





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

        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);

    }
        public boolean intakeWheelOn() {
        intakeWheel.setPower(1);
        return true;
    }

    public boolean intakeWheelOff() {
        intakeWheel.setPower(0);
        return true;
    }
}

