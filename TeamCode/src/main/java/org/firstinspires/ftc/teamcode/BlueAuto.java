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
            hdw.moveToXYPosition(-1200,0,0 );

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return;
    }

    @Override
    void moveToShootingPosition() {
        try {
            hdw.moveToXYPosition(0,0,45 );
            hdw.moveByAprilTag(tagId,1500);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


}
