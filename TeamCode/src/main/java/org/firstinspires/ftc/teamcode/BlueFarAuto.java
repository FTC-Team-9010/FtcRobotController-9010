package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueFarAuto")
public class BlueFarAuto extends BaseAuto2026{
    public BlueFarAuto() {
        tagId = 20;
    }

    @Override
    void moveToReadPosition() {
        try {
            hdw.moveToXYPosition(-1600,200, 0);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return;
    }

    @Override
    void moveToShootingPosition() {
        try {
            hdw.moveToXYPosition(0,0,45 );
            hdw.moveByAprilTag(tagId,1400);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    void movePark() {

    }

    @Override
    void moveToIntake() {
        try {
            hdw.moveByAprilTag(tagId, 1600);
            hdw.moveToXYPosition(0,0, -135);
            hdw.moveToXYPosition(700, 0, 0);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    void moveAfterShoot() {
        try {
            hdw.moveToXYPosition(-400, -700, 45);
            hdw.moveByAprilTag(tagId,1400);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

}
