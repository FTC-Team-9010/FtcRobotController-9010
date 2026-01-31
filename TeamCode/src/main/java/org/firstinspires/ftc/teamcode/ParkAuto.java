package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move/ParkAuto")
public class ParkAuto extends BaseAuto2026{
    public ParkAuto() {
        tagId = 20;
    }

    @Override
    void moveToReadPosition() {

    }

    @Override
    void moveToShootingPosition() {

    }

    @Override
    void movePark() {
        try {
            hdw.moveToXYPosition(400, 0, 0);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    void moveToIntake() {

    }

    @Override
    void moveAfterShoot() {

         }

}
