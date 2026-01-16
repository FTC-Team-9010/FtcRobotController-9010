package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;
import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.stream.IntStream;

/**
 * This class controls Carouel.
 * Carouel has a meg detector, to initialize the position of the carousel.
 * After this step. following movement is by motor encoder.   360  is about 538 encoder tick.
 *
 * There are 3 color sensor on carouel.  Position 0 is where launcher is.  Counting clockwise, is
 * Position 1 and position 2.
 *
 */
public class CarouelController {

    /**
     * The status of ball. Either a Green, or Purple, or empty slot.
     */
    private final int GREEN = 10;
    private final int PURPLE = 1;
    private final int EMPTY = 0;

    /**
     * Flag to show if a read of config is necessary.
     * 1. All 3 are empty.
     * 2. Manual trigger.
     */
    private boolean configReadNeeded =true;

    /**
     * The configuration of balls on the carouel .
     * 0 - Luncher.  Clockwise position 1 and 2.
     */
    private int[] ballConfiguration = new int[3];

    /**
     * PID controller parameters for carouel.
     */
    private double turnKP = 7;
    private double turnKI = .8;
    private double turnKD = 0.002;
    private double turnKF = 0.0;

    private boolean intakeMode = false;
    private final int intakeOffset = 56;
    HardwareMap hardwareMap;

    NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[3];

    private DcMotor launcher = null;
    private float launcherPower = 0;

    public float presetLaunchPower = (float) 1 ;
    private Servo lanchLever = null;
    //Non-cube bot private final float leverLowPosition =(float) 1;
    private final float leverLowPosition = (float) 0.0;

    public DcMotorEx carouel = null;
    public RevTouchSensor meg = null;

    public float getLauncherPower() {
        return launcherPower;
    }

    public void setLauncherPower (float power ) {
        launcherPower=power;
        launcher.setPower(power);
    }

    public void raiseLever ( ) {
        // Non-cube bot setting lanchLever.setPosition(0.3);
        lanchLever.setPosition(.25);
    }

    public void lowerLever() {
        lanchLever.setPosition(leverLowPosition);
    }

    public double getTurnKD() {
        return turnKD;
    }

    public void setTurnKD(double turnKD) {
        this.turnKD = turnKD;
    }

    public double getTurnKI() {
        return turnKI;
    }

    public void setTurnKI(double turnKI) {
        this.turnKI = turnKI;
    }

    public double getTurnKP() {
        return turnKP;
    }

    public void setTurnKP(double turnKP) {
        this.turnKP = turnKP;
    }

    public int[] getBallConfiguration() {
        return ballConfiguration;
    }

    /**
     * Encoder counter for 360 degress.
     */
    private final int oneCircle = 538;

    private Telemetry tel;

    public CarouelController(HardwareMap hwMap , Telemetry telemetry) {
        hardwareMap = hwMap;
        tel = telemetry;
    }

    /**
     * Initialize the hardware of Carouel
     */
    public void initialize() {
        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensors[0] = hardwareMap.get(NormalizedColorSensor.class, "color0");
        colorSensors[1] = hardwareMap.get(NormalizedColorSensor.class, "color1");
        colorSensors[2] = hardwareMap.get(NormalizedColorSensor.class, "color2");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can)
        for (int i = 0; i < 3; i++) {
            if (colorSensors[i] instanceof SwitchableLight) {
                ((SwitchableLight) colorSensors[i]).enableLight(true);
            }
            float gain = 2;
            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            colorSensors[i].setGain(gain);
        }

        //Initialize the motor.
        carouel = hardwareMap.get(DcMotorEx.class, "carousel");
        carouel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouel.setVelocity(0);

        //Init Magenetic limit switch.
        meg = hardwareMap.get(RevTouchSensor.class, "meg");

        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        lanchLever = hardwareMap.get(Servo.class,"launchLever");
        lanchLever.setPosition(leverLowPosition);
    }

    /**
     * Using the magetic senstor to move the carouel to init position.
     * Then reset encoder to 0 on the motor.
     */


    public void initPosition() {

        int currentPosition = carouel.getCurrentPosition();
        Log.d("9010:", "Carousel Current Position: " + currentPosition);

        if (meg.isPressed()) {
            return;
        }

        //Start spining
        carouel.setVelocity(-100);
        while (!meg.isPressed()) {
            currentPosition = carouel.getCurrentPosition();
            Log.d("9010","Position with MagLimit: " + currentPosition);
        }
        carouel.setVelocity(0);


        try {
            Thread.sleep(500);
        } catch (Exception ex) {
            //DO nothing.
            Log.d("9010", ex.toString());
        }
        //Start spining in reverse, but slower.
        carouel.setVelocity(50);

        while (!meg.isPressed()) {
            currentPosition = carouel.getCurrentPosition();
            Log.d("9010","Position with MagLimit backward: " + currentPosition);
        }
        carouel.setVelocity(0);

        currentPosition = carouel.getCurrentPosition();
        Log.d("9010", "Position after stop: " + currentPosition);

        //Move antoher angle for the launch position
        moveToPosition(currentPosition-72 , 3 );

        carouel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void alignIntake() {
        intakeMode = true;
        int currentPos = carouel.getCurrentPosition();
        moveToPosition( currentPos - intakeOffset, 3  );
    }

    public void alignShoot() {
        intakeMode = false;
        int currentPos = carouel.getCurrentPosition();
        moveToPosition( currentPos + intakeOffset, 3  );
    }

    public boolean isIntakeMode() {
        return intakeMode;
    }

    public void setIntakeMode(boolean intakeMode) {
        this.intakeMode = intakeMode;
    }

    /**
     * Rotate clock wise for 120 degrees.
     */

    
    public void rotateOneSlotCW() {

        int startPosition = carouel.getCurrentPosition();
        //Figure out if initial position inside which slot.
        //Slot 1 : 0,  Slot 1:  179,  Slot 2: 538
        int currentSlotNumber = (int) Math.round((float) startPosition / (oneCircle/3));
        int regulatedCurrentPosition = currentSlotNumber * (oneCircle / 3);
        if ( intakeMode) {
            regulatedCurrentPosition-=intakeOffset;
        }
        Log.d("9010", " start Position: " + startPosition + "Regulated start: " + regulatedCurrentPosition);

        int targetPosition = regulatedCurrentPosition + oneCircle / 3;
        moveToPosition(targetPosition, 3);
        int temp = ballConfiguration[0];
        ballConfiguration[0] = ballConfiguration[2];
        ballConfiguration[2] = ballConfiguration[1];
        ballConfiguration[1] = temp;

        tel.addData("Ball Config: [" , ballConfiguration[0] + "] [" +
                ballConfiguration[1] + "] ["+ ballConfiguration[2]+"]");
        tel.update();
    }

    /**
     * Rotate counter clock wise for 120 degrees.
     */
    public void rotateOneSlotCCW() {
        int startPosition = carouel.getCurrentPosition();
        //Figure out if initial position inside which slot.
        //Slot 1 : 0,  Slot 1:  179,  Slot 2: 538
        int currentSlotNumber = (int) Math.round((float) startPosition / (oneCircle/3));
        int regulatedCurrentPosition = currentSlotNumber * (oneCircle / 3);
        if ( intakeMode) {
            regulatedCurrentPosition-=intakeOffset;
        }
        //Log.d("9010", " start Position: " + startPosition + "Regulated start: " + regulatedCurrentPosition);

        int targetPosition = regulatedCurrentPosition - oneCircle / 3;
        moveToPosition(targetPosition, 3);

        int temp = ballConfiguration[0];
        ballConfiguration[0] = ballConfiguration[1];
        ballConfiguration[1] = ballConfiguration[2];
        ballConfiguration[2] = temp;

        tel.addData("Ball Config: [" , ballConfiguration[0] + "] [" +
                ballConfiguration[1] + "] ["+ ballConfiguration[2]+"]");
        tel.update();
    }

    public void rotateNudgeCW() {
        int startPosition = carouel.getCurrentPosition();
        int targetPosition = carouel.getCurrentPosition() + 20;
        moveToPosition(targetPosition, 3);
    }

    public void rotateNudgeCCW() {
        int startPosition = carouel.getCurrentPosition();
        int targetPosition = carouel.getCurrentPosition() - 20;
        moveToPosition(targetPosition, 3);
    }



    /**
     * Move to position by motor encoder click.
     *
     * @param targetPosition Target postion
     * @param tolerance      Torrence, larger the tolerance, less accurate.
     */
    private void moveToPosition(int targetPosition, int tolerance) {
        PIDFController turnPidfCrtler = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        //Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);
        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(tolerance);
        turnPidfCrtler.setIntegrationBounds(-20, 20);

        while (!turnPidfCrtler.atSetPoint()) {
            double calculatedV = -turnPidfCrtler.calculate(targetPosition - carouel.getCurrentPosition());
            //Log.d("9010","calV: " + calculatedV + " pos: " + carouel.getCurrentPosition());
            carouel.setVelocity(calculatedV);
        }
        carouel.setVelocity(0);
        //Log.d("9010", "Position after turn: " + carouel.getCurrentPosition());
    }


    /**
     * Get the HSV value of color sensor N
     *
     * @param sensorId Id of sensor.
     * @return the color of HSV
     */
    private float getHsv(int sensorId) {

        int currentPosition = carouel.getCurrentPosition();
        int moveNum = 30;
        double distance = ((DistanceSensor) colorSensors[sensorId]).getDistance(DistanceUnit.CM);
        Log.d("9010", "Distance is: " + distance);
        //If Distance is larger than 4 CM, reading of color is unreliable
        boolean moveFlag = false;
        if (distance > 4) {
            //First rotate the carouel 6 ticks
            moveToPosition(moveNum + currentPosition, 10);
            moveFlag = true;
        }

        //Get distance again.
        distance = ((DistanceSensor) colorSensors[sensorId]).getDistance(DistanceUnit.CM);
        if (distance > 4) {
            if (moveFlag) {
                moveToPosition(currentPosition, 3);
            }
            return 0;
        } else {
            final float[] hsvValues = new float[3];
            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensors[sensorId].getNormalizedColors();

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);
            float colorValue = hsvValues[0];
            if (moveFlag) {
                moveToPosition(currentPosition, 3);
            }
            return colorValue;
        }
    }

    /**
     * Read all 3 colors into the ball configuration.
     * after reading, set confReadNeeded flag to false;
     */

    public void readBallConfiguration() {
        for (int i = 0; i < 3; i++) {
            float color = this.getHsv(i);
            Log.d("9010", "Color value: " + color);
            if (color > 200) {
                ballConfiguration[i] = PURPLE;
            } else if (color <= 200 && color > 0) {
                ballConfiguration[i] = GREEN;
            } else if (color == 0) {
                ballConfiguration[i] = EMPTY;
            }
            tel.addData("Ball Config: [" , ballConfiguration[0] + "] [" +
                    ballConfiguration[1] + "] ["+ ballConfiguration[2]+"]");
            tel.update();
        }
        if (ballConfiguration[0] + ballConfiguration[1]+ ballConfiguration[2] ==0 ){
            configReadNeeded = true;
        } else {
            configReadNeeded = false;
        }
    }



    /**
     * match the ball configuration to the sequence, by rotating the carouel.
     * After matching, the sequence of ball in position 0 , 1, 2  shall match
     * the sequence given where the green ball shall be at.
     *
     * @param targetGreenIndex The index where the green ball shall be.
     * @return true if config can match. Other wise retrun false.
     */
    public boolean matchConfigToSequence(int targetGreenIndex) {
        //1.  Check if ball configration is 2 purple and 1 green.
        int sum = IntStream.of(ballConfiguration).sum();
        boolean ret = false;
        Log.d("9010", "Sum is : " + sum + " targetGreenIndex: " + targetGreenIndex);
        if (sum != 12) {
            ret = false;
        } else {
            //Find out green index in the ball config.
            int cGreenIndex = 0;
            for (int i = 0; i < 3; i++) {
                if (ballConfiguration[i] == GREEN) {
                    cGreenIndex = i;
                }
            }
            Log.d("9010","Found green in : " + cGreenIndex);
            //Calculate the difference between green index and target green index.
            int diff = cGreenIndex - targetGreenIndex;
            if (diff == 0) {
                // We already match.
                ret = true;
            } else if (diff == -1 || diff == 2) {
                rotateOneSlotCW();
                ret = true;
            } else if (diff == 1 || diff == -2) {
                rotateOneSlotCCW();
                ret = true;
            }
        }
        return ret;
    }


    /**
     * Shoot the ball in the pattern.
     * This method will call read Ball COnfiguration, and then match configToSequence.
     * THen,  lunch Ball in detected sequence.
     */

    public void shootPattern(int decodedGreenIndex) {
        if ( configReadNeeded ) {
            readBallConfiguration();
        }
        if (matchConfigToSequence(decodedGreenIndex)) {
            Log.d("9010", ballConfiguration[0] + " " + ballConfiguration[1] + " " + ballConfiguration[2]);
            shootBall();
            Log.d("9010", "after first shoot " );
            for (int i = 1; i < 3; i++) {
                rotateOneSlotCCW();
                shootBall();
                //Log.d("9010", " loop shoot " + ballConfiguration[0] + " " + ballConfiguration[1] + " " + ballConfiguration[2]);
            }
        }

    }



    /**
     * This method shoot the ball in position 0 .
     * After shooting, set ball position 0 to empty.
     */
    public void shootBall() {
        try {
            //Sping launcher
            //setLauncherPower(presetLaunchPower);
            raiseLever();
            Thread.sleep(700);
            lowerLever();
            //setLauncherPower(0);

            //After shoot ball, position 0 becomes empty
            ballConfiguration[0]=EMPTY;
            Log.d("9010", "After shoot ball, ball config: "
                    + ballConfiguration[0] + " " + ballConfiguration[1] + " " + ballConfiguration[2]);
            if ( ballConfiguration[1]==EMPTY  && ballConfiguration[2]==EMPTY ) {
                //All 3 empty, set read flag.
                configReadNeeded = true;
            }
            tel.addData("Ball Config: [" , ballConfiguration[0] + "] [" +
                    ballConfiguration[1] + "] ["+ ballConfiguration[2]+"]");
            tel.addData("read needed:" , configReadNeeded);
            tel.update();

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }



    public void shootGreen() {
        if (configReadNeeded) {
            readBallConfiguration();
        }
        int carGreenIndex = -1;
        for (int i = 0; i < 3; i++) {
            if (ballConfiguration[i] == GREEN) {
                carGreenIndex = i;
            }
        }
        //If not found green, return
        if (carGreenIndex==-1) {
            Log.d("9010","Did not found Green ball");
            return;
        }

        if (carGreenIndex == 0) {
            // We already match.
            //Log.d("9010", "match");
        } else if (carGreenIndex == 2) {
            rotateOneSlotCW();
        } else if (carGreenIndex == 1) {
            rotateOneSlotCCW();
        }
        shootBall();
        //Log.d("9010", ballConfiguration[0] + " " + ballConfiguration[1] + " " + ballConfiguration[2]);
    }

    public void shootPurple() {
        if (configReadNeeded) {
            readBallConfiguration();
        }
        int carPurpleIndex = -1;
        for (int i = 0; i < 3; i++) {
            if (ballConfiguration[i] == PURPLE) {
                carPurpleIndex = i;
            }
        }
        //If not found green, return
        if (carPurpleIndex==-1) {
            Log.d("9010","DId not found Purple ball ");
            return;
        }

        if (carPurpleIndex == 0) {
            // We already match.
            //Log.d("9010", "match");
        } else if (carPurpleIndex == 2 ) {
            rotateOneSlotCW();
        } else if (carPurpleIndex == 1) {
            rotateOneSlotCCW();
        }
        shootBall();
    }

}
