package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous(name = "RightAuto")
public class RightAuto extends BaseAuto {

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
            //Move back 18 in
            hdw.moveToXYPosition(-457, 0, 0 );

            //Move right 60 in
            hdw.moveToXYPosition( 0 ,  - 950 , 0 );
            //hdw.moveToXYPosition(-150,0,0);

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }


}