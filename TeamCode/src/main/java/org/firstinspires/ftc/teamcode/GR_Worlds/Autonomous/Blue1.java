package org.firstinspires.ftc.teamcode.GR_Worlds.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Blue 1", group = "Sensor")
public class Blue1 extends WorldsMasterAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        declare();
        initGyro();
        waitForStart();
        initVuforia();
        setBaseAngles("blue1");

        waitForStart();

        while (opModeIsActive()) {
            RelicRecoveryVuMark column = getPicto();
            jewelSequence("blue");
            sleep(200);
            // TODO from abby: 4/12/18 test these numbers:
            moveTicksForward(.4, 2000);
            sleep(200);
            placeGlyphSequence(column);
            getMoreGlyphsStone1();
            // TODO from abby: 4/15/18  test this function I wrote it at home and all the values are likely very wrong:
            placeSecondGlyphSequence(column);
            break;
        }
    }
}