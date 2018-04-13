package org.firstinspires.ftc.teamcode.GR_Worlds.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "NEW Red 2", group = "Sensor")
public class Red2 extends WorldsMasterAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        declare();
        initGyro();
        waitForStart();
        initVuforia();
        setStartAngle();
        waitForStart();

        while (opModeIsActive()) {
            RelicRecoveryVuMark column = getPicto();
            //jewelSequence("red");
            delay(200);
            // TODO from abby: 4/12/18 change to encoder:
            moveTicksBack(.4,1000);
            delay(950);
            moveTicksRight(.6, 900);
            delay(200);
            // TODO from abby: 4/12/18 test this angle:
            turnToColumnSequence(column, 0);
            placeGlyphSequence(column);
            break;
        }
    }
}