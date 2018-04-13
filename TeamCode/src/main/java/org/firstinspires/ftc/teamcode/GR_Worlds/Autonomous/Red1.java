package org.firstinspires.ftc.teamcode.GR_Worlds.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "NEW Red 1", group = "Sensor")
public class Red1 extends WorldsMasterAuto {

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
            jewelSequence("red");
            delay(200);
            // TODO from abby: 4/12/18 change to encoder driving:
            moveBackward(.4,850);
            delay(200);
            // TODO from abby: 4/12/18 test this angle:
            turnToColumnSequence(column, 90);
            placeGlyphSequence(column);
            break;
        }
    }
}