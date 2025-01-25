package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hardware2025Tifini {

    //TODO: Recalculate these with new wheels
    private final double xAxisCoeff = 216.5;  // How many degrees encoder to turn to run an inch in X Axis
    private final double yAxisCoeff = 216.5;  // How many degrees encoder to turn to run an inch in Y Axis

    private boolean debug = true;
    private Telemetry telemetry;

    public Hardware2025Tifini(HardwareMap m, Telemetry tm) {
        hwMap = m;
        telemetry = tm;
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

    public void intakeWheelOn() {
        intakeWheel.setPower(1);
    }

    public void intakeWheelOff() {
        intakeWheel.setPower(0);
    }
}

