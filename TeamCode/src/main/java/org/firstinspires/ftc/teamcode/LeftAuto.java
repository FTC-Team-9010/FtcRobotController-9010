package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LeftAuto")
public class LeftAuto extends BaseAuto {
    @Override
    void moveBeforeSpecimen() {
        //Move right 5 in
        try {
            hdw.moveToXYPosition( 0 ,  -130 , 0 );
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    void park ( ) {
        try {
            //Move right 50 in
            hdw.moveToXYPosition( 0 ,   -1300, 0 );

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}