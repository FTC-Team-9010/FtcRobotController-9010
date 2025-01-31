package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LeftNoPark")
public class LeftNoParkAuto extends BaseAuto{
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

    }

}
