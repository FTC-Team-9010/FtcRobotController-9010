package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.util.Log;
@Autonomous(name = "RightPickup_Auto")
public class RightPickupAuto extends BaseAuto {

    @Override
    void moveBeforeSpecimen() {
        //Move left 5 in
        try {
            hdw.moveToXYPosition(450,0,0);
            hdw.moveToXYPosition( 0 ,  130 , 0 );
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    void park ( )  {
        try {

            hdw.moveToXYPosition(0, -780, 0);
            hdw.moveToXYPosition(762, 0 , 0);
            hdw.moveToXYPosition(0,0,90);
            hdw.moveToXYPosition(-304, 0,0);
            hdw.moveToXYPosition(0,1050,0);

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }


}
