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
            hdw.moveToXYPosition(-1200,0,0 );

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return;
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
}
