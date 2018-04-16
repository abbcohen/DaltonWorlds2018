package org.firstinspires.ftc.teamcode.GR_Worlds.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Red 2", group = "Sensor")
public class Red2 extends WorldsMasterAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        declare();
        initGyro();
        waitForStart();
        initVuforia();
        setBaseAngles("red2");
        waitForStart();

        while (opModeIsActive()) {
            RelicRecoveryVuMark column = getPicto();
            jewelSequence("red");
            sleep(200);
            moveTicksBack(.4,1200);
            sleep(950);
            moveTicksRight(.6, 800);
            sleep(200);
            placeGlyphSequence(column);
            // TODO from abby: 4/15/18  test this function I wrote it at home and all the values are likely very wrong:
            getMoreGlyphsStone2("red");
            placeSecondGlyphSequence(column);
            break;
        }
    }
}