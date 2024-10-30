package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RightAuto")
public class RightAuto extends BaseAuto {

    void park ( )  {

        try {
            //Move to right 15 in
            hdw.moveToXYPosition(-381  ,  0 , 0 );
            //Move up 55 in
            hdw.moveToXYPosition(0,1400, 0 );
            //MOve to left 14 in
            hdw.moveToXYPosition(355, 0, 0 );

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}