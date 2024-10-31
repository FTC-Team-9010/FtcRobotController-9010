package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RightAuto")
public class RightAuto extends BaseAuto {

    void park ( )  {
        try {
            //Move to right 48 in
            hdw.moveToXYPosition( 0 ,  -1219  , 0 );

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}