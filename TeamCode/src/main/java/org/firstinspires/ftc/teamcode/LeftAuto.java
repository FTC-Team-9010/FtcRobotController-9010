package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LeftAuto")
public class LeftAuto extends BaseAuto {
    @Override
    void moveBeforeSpecimen() {

    }

    void park ( ) {
        try {

            //Move back 25 in
            hdw.moveToXYPosition(-625, 0, 0 );
            //Move right 50 in
            hdw.moveToXYPosition( 0 ,   1300, 0 );

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}