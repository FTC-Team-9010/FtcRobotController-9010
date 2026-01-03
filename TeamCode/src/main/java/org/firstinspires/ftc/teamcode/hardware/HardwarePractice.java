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
    public float launcherPower;

    public HardwarePractice(HardwareMap m, Telemetry tm) {
        hwMap = m;
        telemetry = tm;
    }

    static public double ANGULAR_RATE = 2500.0;
    private final double MIN_VELOCITY = 0.1;

    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;

    public DcMotorEx launcher = null;

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

    public void setLauncherPower(float power) {
        launcherPower = power;
        launcher.setPower(power);
    }
}