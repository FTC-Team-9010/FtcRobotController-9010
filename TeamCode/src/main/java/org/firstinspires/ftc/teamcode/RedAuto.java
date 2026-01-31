package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAuto")
public class RedAuto extends BaseAuto2026 {

    //TODO: Mirror values from BlueFarAuto
    public RedAuto() {
        tagId = 24 ;
    }

    @Override
    void moveToReadPosition() {
        try {
            hdw.moveToXYPosition(0,-600,45 );

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    void moveToShootingPosition() {
        try {
            hdw.moveToXYPosition(0,0,-90 );
            hdw.moveToXYPosition(600, 0, 0);
            hdw.moveByAprilTag(tagId,hdw.shootingRange);

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
            hdw.moveToXYPosition(0,0, 135);
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
