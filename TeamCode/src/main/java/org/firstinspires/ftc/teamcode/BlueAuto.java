package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAuto")
public class BlueAuto extends BaseAuto2026{
    public BlueAuto() {
        tagId = 20;
    }

    @Override
    void moveToReadPosition() {
        try {
            hdw.moveToXYPosition(0,600,-45 );

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return;
    }

    @Override
    void moveToShootingPosition() {
        try {
            hdw.moveToXYPosition(0,0,90 );
            hdw.moveToXYPosition(600, 0, 0);
            hdw.moveByAprilTag(tagId,1400);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    void movePark() {

    }


}
