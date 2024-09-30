package org.firstinspires.ftc.teamcode.hardware;


//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hardware2024Fred {

    public HardwareMap hwMap;


    //servos
    //motors
    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;
    public DcMotorEx vSlide = null;
    public DcMotorEx elevation = null;

    //IMU
    public IMU imu = null;

    //This is max wheel and slide motor velocity.
    static public double ANGULAR_RATE = 2500.0;

    private final double xAxisCoeff = 216.5;  // How many degrees encoder to turn to run an inch in X Axis
    private final double yAxisCoeff = 216.5;  // How many degrees encoder to turn to run an inch in Y Axis

    private boolean debug = true;
    private Telemetry telemetry;

    private double slideUpperLimit = 2000;

    //PID control parameter for turning.
    private double turnKP = 0.15;
    private double turnKI = 0.1;
    private double turnKD = 0.005;
    private double turnKF = 0.0;

    private double lnKP = 1.5;
    private double lnKI = 0.15;
    private double lnKD = 0.11;
    private double lnKF = 0.0;

    private double slideKP = 1.27;
    private double slideKI = 0.03;
    private double slideKD = 0.001;


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
    }


    private int vsldieInitPosition = 0;
    /**
     * Initialize hardware.
     */
    public void createHardware() {
        wheelFrontRight = hwMap.get(DcMotorEx.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "lrWheel");
        vSlide = hwMap.get(DcMotorEx.class, "vSlideM");
        elevation = hwMap.get(DcMotorEx.class, "elev");

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

        vSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vsldieInitPosition = vSlide.getCurrentPosition() ;

        //Get IMU.
        imu = hwMap.get(IMU.class, "imu");

        //Our robot mount Control hub Logo face backward, and USB port is facing Up.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));


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


    public void freeMoveVerticalSlide(float power ) {
        double slidePosition  = vSlide.getCurrentPosition();

        //Control  Vslide
        if ((power > 0 && slidePosition < slideUpperLimit) || (power < 0 && slidePosition > 0)) {
            vSlide.setVelocity(power * ANGULAR_RATE);
        } else {
            vSlide.setVelocity(0);
        }

    }
    public void elevation(float power) {
        double slidePosition  = elevation.getCurrentPosition();
        if ((power > 0 && slidePosition < slideUpperLimit) || (power < 0 && slidePosition > 0)) {
            elevation.setVelocity(power * ANGULAR_RATE);

        } else {
            elevation.setVelocity(0);

        }
    }


}