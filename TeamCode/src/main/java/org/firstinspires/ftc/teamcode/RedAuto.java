package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAuto")
public class RedAuto extends  BaseAuto2026 {

    public RedAuto() {
        tagId = 24 ;
    }

    @Override
    void moveToReadPosition() {
        try {
            hdw.moveToXYPosition(800,-1500,0 );

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    void moveToShootingPosition() {
        try {
            hdw.moveToXYPosition(0,0,-45 );
            hdw.moveByAprilTag(tagId,1900);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    void movePark() {
        try {
            hdw.moveToXYPosition(0, 909, 0);
        } catch (InterruptedException e ) {
            throw new RuntimeException(e);
        }
    }

}
