package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;
import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Collection;
import java.util.stream.IntStream;

public class CarouelController {

    /**
     * The status of ball. Either a Green, or Purple, or empty slot.
     */
    private final int GREEN = 10 ;
    private final int PURPLE = 1;
    private final int EMPTY = 0;


    /**
     * The configuration of balls on the carouel .
     */
    private int [] ballConfiguration = new int [3];


    /**
     * PID controller parameters for carouel.
     */
    private double turnKP = 7;
    private double turnKI = .8;
    private double turnKD = 0.002;
    private double turnKF = 0.0;

    HardwareMap hardwareMap;


    NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[3];

    public DcMotorEx carouel = null;
    public RevTouchSensor meg  = null;

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
    public int [] getBallConfiguration() {
        return ballConfiguration;
    }

    /**
     * Encoder counter for 360 degress.
     */
    private int oneCircle = 538 ;


    public CarouelController(HardwareMap hwMap) {
        hardwareMap = hwMap;
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
        carouel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouel.setVelocity(0);

        //Init Magenetic limit switch.
        meg = hardwareMap.get(RevTouchSensor.class, "meg");

    }

    /**
     * Using the magetic senstor to move the carouel to init position.
     * Then reset encoder to 0 on the motor.
     */
    public void initPosition() {

        int currentPosition = carouel.getCurrentPosition();
        Log.d("9010:", "Carousel Current Position: " + currentPosition);

        if ( meg.isPressed()) {
            return;
        }

        //Start spining
        carouel.setVelocity(-500);
        while (!meg.isPressed()) {
            //currentPosition = carouel.getCurrentPosition();
            //Log.d("9010","Position with MagLimit" + currentPosition);
        }
        carouel.setVelocity(0);

        try {
            Thread.sleep(500);
        } catch (Exception ex) {
            //DO nothing.
            Log.d("9010", ex.toString());
        }
        //Start spining in reverse, but slower.
        carouel.setVelocity(80);

        while (!meg.isPressed()) {
            //currentPosition = carouel.getCurrentPosition();
            //Log.d("9010","Position with MagLimit backward" + currentPosition);
        }
        carouel.setVelocity(0);

        Log.d("9010", "Position after stop: " + carouel.getCurrentPosition());

        carouel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Rotate clock wise for 120 degrees.
     */
    public void rotateOneSlotCW() {

        int startPosition =  carouel.getCurrentPosition();
        //Figure out if initial position inside which slot.
        //Slot 1 : 0,  Slot 1:  179,  Slot 2: 538
        int currentSlotNumber =(int) Math.round ( (float) startPosition/179 );
        int regulatedCurrentPosition = currentSlotNumber* (oneCircle/3);
        Log.d("9010",  " start Position: " + startPosition + "Regulated start: " + regulatedCurrentPosition);

        int targetPosition =  regulatedCurrentPosition +  oneCircle/3  ;
        moveToPosition(targetPosition,3);
    }

    /**
     * Rotate counter clock wise for 120 degrees.
     */
    public void rotateOneSlotCCW() {
        int startPosition =  carouel.getCurrentPosition();
        //Figure out if initial position inside which slot.
        //Slot 1 : 0,  Slot 1:  179,  Slot 2: 538
        int currentSlotNumber =(int) Math.round ( (float) startPosition/179 );
        int regulatedCurrentPosition = currentSlotNumber* (oneCircle/3);
        Log.d("9010",  " start Position: " + startPosition + "Regulated start: " + regulatedCurrentPosition);

        int targetPosition =  regulatedCurrentPosition -  oneCircle/3  ;
        moveToPosition(targetPosition,3);

    }

    /**
     * Move to position by motor encoder click.
     *
     * @param targetPosition  Target postion
     * @param tolerance   Torrence, larger the tolerance, less accurate.
     */
    private void moveToPosition(int targetPosition, int tolerance ) {
        PIDFController turnPidfCrtler = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);
        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(tolerance);
        turnPidfCrtler.setIntegrationBounds(-20, 20);

        while ( !turnPidfCrtler.atSetPoint()) {
            double calculatedV = - turnPidfCrtler.calculate(targetPosition - carouel.getCurrentPosition());
            //Log.d("9010","calV: " + calculatedV + " pos: " + carouel.getCurrentPosition());
            carouel.setVelocity(calculatedV);
        }
        carouel.setVelocity(0);

        Log.d("9010","Position after turn: " + carouel.getCurrentPosition());

    }


    /**
     * Get the HSV value of color sensor N
     * @param sensorId Id of sensor.
     * @return the color of HSV
     */
    private float getHsv(int sensorId ) {

        int currentPosition = carouel.getCurrentPosition();
        int moveNum = 30;
        double distance = ((DistanceSensor) colorSensors[sensorId]).getDistance(DistanceUnit.CM);
        Log.d("9010", "Distance is: " + distance);
        //If Distance is larger than 4 CM, reading of color is unreliable
        boolean moveFlag = false;
        if ( distance> 4 ) {
            //First rotate the carouel 6 ticks
            moveToPosition(moveNum + currentPosition,10);
            moveFlag = true;
        }

        //Get distance again.
        distance = ((DistanceSensor) colorSensors[sensorId]).getDistance(DistanceUnit.CM);
        if ( distance> 4 ){
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
     */
    public void readBallConfiguration() {
        for ( int i = 0; i < 3 ; i ++ ) {
            float color = this.getHsv(i);
            Log.d("9010", "Color value: " + color);
            if ( color> 200 ) {
                ballConfiguration[i] =  PURPLE;
            } else if ( color <=200 && color > 0 ) {
                ballConfiguration[i] = GREEN;
            } else if ( color == 0 ) {
                ballConfiguration[i] = EMPTY;
            }
            Log.d("9010", "Ball Configuration for sensor " + i + " : " + ballConfiguration[i]);
        }
    }

    /**
     * match the ball configuration to the sequence, by rotating the carouel.
     * @param targetGreenIndex  The index where the green ball shall be.
     * @return  true if config can match. Other wise retrun false.
     */
    public boolean  matchConfigToSequence ( int targetGreenIndex ) {
        //1.  Check if ball configration is 2 purple and 1 green.
        int sum = IntStream.of(ballConfiguration).sum();
        boolean ret = false;
        Log.d("9010", "Sum is : " + sum + " targetGreenIndex: " + targetGreenIndex);
        if ( sum !=12 ) {
            ret =  false ;
        } else {
            //Find out green index in the ball config.
            int cGreenIndex = 0;
            for ( int i=0; i< 3 ; i++) {
                if ( ballConfiguration[i] == GREEN ) {
                    cGreenIndex = i;
                }
            }
            //Calculate the difference between green index and target green index.
            int diff = cGreenIndex - targetGreenIndex;
            if ( diff == 0 ) {
                // We already match.
                ret =  true;
            } else if ( diff ==-1 || diff == 2 ) {
                rotateOneSlotCW();
                ret =  true;
            } else if ( diff == 1 || diff == -2 ) {
                rotateOneSlotCCW();
                ret =  true;
            }
        }
        return ret;
    }

}
