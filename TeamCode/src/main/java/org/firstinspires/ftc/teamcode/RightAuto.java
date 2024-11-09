package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RightAuto")
public class RightAuto extends BaseAuto {

    @Override
    void moveBeforeSpecimen() {

    }

    @Override
    void park ( )  {
        try {
            //Move back 25 in
            hdw.moveToXYPosition(-625, 0, 0 );
            //Move right 60 in
            hdw.moveToXYPosition( 0 ,  -1524  , 0 );

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}