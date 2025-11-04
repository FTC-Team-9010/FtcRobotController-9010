package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous(name = "LeftAuto")
public class LeftAuto extends BaseAuto {
    @Override
    void moveBeforeSpecimen() {
        //Move right 5 in
        try {
            hdw.moveToXYPosition(450,0,0);
            hdw.moveToXYPosition( 0 ,  -130 , 0 );
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    void park ( ) {
        try {
            //Move back 18 in
            hdw.moveToXYPosition(-457, 0, 0 );

            //Move right 50 in
            hdw.moveToXYPosition( 25 ,   -1300, 0 );
            //hdw.moveToXYPosition(-150,0,0);

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}