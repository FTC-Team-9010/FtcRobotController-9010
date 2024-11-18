package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RightAuto")
public class RightAuto extends BaseAuto {

    @Override
    void moveBeforeSpecimen() {
        //Move left 5 in
        try {
            hdw.moveToXYPosition( 0 ,  130 , 0 );
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    void park ( )  {
        try {

            //Move right 60 in
            hdw.moveToXYPosition( 0 ,  - 900 , 0 );

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}