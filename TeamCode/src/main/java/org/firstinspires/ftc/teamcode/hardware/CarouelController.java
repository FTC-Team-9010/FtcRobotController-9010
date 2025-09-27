package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class CarouelController {

    HardwareMap hardwareMap;

    public CarouelController(HardwareMap hwMap) {
        hardwareMap = hwMap;
    }

    NormalizedColorSensor colorSensor1;
    public DcMotorEx carouel = null;
    public RevTouchSensor meg  = null;

    /**
     * Encoder counter for 360 degress.
     */
    private int oneCircle = 540 ;
    /**
     * Initialize the hardware of Carouel
     */
    public void initialize() {
        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "color1");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor1).enableLight(true);
        }
        float gain = 2;
        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor1.setGain(gain);


        carouel = hardwareMap.get(DcMotorEx.class, "carousel");
        carouel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouel.setVelocity(0);

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
            currentPosition = carouel.getCurrentPosition();
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
            currentPosition = carouel.getCurrentPosition();
            Log.d("9010","Position with MagLimit backward" + currentPosition);
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
        int targtPosition =  startPosition + oneCircle/3 ;
        carouel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while ( carouel.isBusy()) {
            carouel.setPower( 0.5);
        }
        carouel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouel.setPower(0);

    }

    private void rotateOneSlotCCW() {

    }



    private float getHsv() {

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];


        // Loop until we are asked to stop

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues[0];
    }

}
