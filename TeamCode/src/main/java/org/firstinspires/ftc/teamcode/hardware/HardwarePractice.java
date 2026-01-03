package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HardwarePractice {
    public HardwareMap hwMap;

    public Telemetry telemetry;

    public HardwarePractice(HardwareMap m, Telemetry tm) {
        hwMap = m;
        telemetry = tm;
    }

    static public double ANGULAR_RATE = 2500.0;
    private final double MIN_VELOCITY = 0.1;

    private float launcherPower = 0;
    public float practicePresetLaunchPower = (float) 1;

    private float flywheelPower = 0;

    public float practiceFlywheelPower = (float) 1;

    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;

    public DcMotorEx launcher = null;
    public DcMotorEx flywheel = null;

    public Servo arm = null;

    public void createHardware() {

        wheelFrontRight = hwMap.get(DcMotorEx.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "lrWheel");

        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheelFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelBackLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelBackRight.setDirection(DcMotor.Direction.FORWARD);

        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        arm = hwMap.get(Servo.class, "arm");

        launcher.setVelocity(0);

        wheelFrontRight.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackLeft.setVelocity(0);

    }

    public void launcherOn() {
        launcher.setPower(launcherPower);
    }

    public void launcherOff() {
        launcher.setPower(0);
    }

    public float getLauncherPower() {
        return launcherPower;
    }

    public void setLauncherPower (float power ) {
        launcherPower=power;
        launcher.setPower(power);
    }

    public void setFlywheelPower(float power) {
        flywheelPower=power;
        flywheel.setPower(power);
    }

    public float getFlywheelPower() {
        return flywheelPower;
    }

    public void armInit() {
        arm.setPosition(0);
    }

    public void armLaunch() {
        arm.setPosition(1);
    }

}