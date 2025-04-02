package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LeftPickup_Auto")
public class LeftPickupAuto extends BaseAuto {
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
            hdw.moveToXYPosition(0, -1082, 0);
            hdw.moveToXYPosition(762, 0 , 0);
            hdw.moveToXYPosition(0,0,90);
            hdw.moveToXYPosition(-304, 0,0);
            hdw.moveToXYPosition(0,1050,0);

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}