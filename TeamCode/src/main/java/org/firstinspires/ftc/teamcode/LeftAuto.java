package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LeftAuto")
public class LeftAuto extends BaseAuto {

    void park ( ) {
        try {
            //Wait 5 seconds.
            Thread.sleep(5000);
            //Move to right 46 in
            hdw.moveToXYPosition( 0 ,  -1150  , 0 );

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}