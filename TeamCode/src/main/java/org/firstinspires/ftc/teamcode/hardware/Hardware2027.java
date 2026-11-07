package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Hardware2027 {

    public HardwareMap hwMap;

    //motors
    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;

    private DcMotorEx luncher = null;
    private DcMotor intake = null;


    private CRServo frIntake = null;
    private CRServo flIntake = null;
    private CRServo inner = null;


    private float intakePower = 0;
    public final float INTAKE_POWER = (float) 1;


    private double moveTimeOut = 5000;

    public Telemetry telemetry;

    static public double ANGULAR_RATE = 2500;

    public Hardware2027(HardwareMap m, Telemetry tm) {
        hwMap = m;
        telemetry = tm;
    }


    public void createHardware() {
        //Initialize Wheels
        wheelFrontRight = hwMap.get(DcMotorEx.class, "frwheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "flwheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "brwheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "blwheel");

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

        //Initialize Luncher and Servos.
        luncher = hwMap.get(DcMotorEx.class, "launcher");
        luncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        luncher.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        frIntake = hwMap.get(CRServo.class,"frintake");
        frIntake.setDirection(CRServo.Direction.FORWARD);
        flIntake = hwMap.get(CRServo.class,"flintake");
        flIntake.setDirection(CRServo.Direction.REVERSE);
        inner = hwMap.get(CRServo.class,"inner");
        inner.setDirection(CRServo.Direction.FORWARD);

    }

    public void setIntakePower( float intakePower) {
        intake.setPower(intakePower);
    }

    public void setLuncherPower ( float luncherPower) {
        luncher.setPower(luncherPower);
    }

    public void innerOn () {
        inner.setPower(1);
    }
    public void innerOff () {
        inner.setPower(0);
    }

    public void intakeSideWheelsOn () {
        frIntake.setPower(1);
        flIntake.setPower(1);
    }

    public void intakeSideWheelsOff () {
        frIntake.setPower(0);
        flIntake.setPower(0);
    }


}
