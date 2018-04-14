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

        while (opModeIsActive() && !isStopRequested()) {
            RelicRecoveryVuMark column = getPicto();
            jewelSequence("red");
            delay(200);
            moveTicksBack(.4,1200);
            delay(950);
            moveTicksRight(.6, 800);
            delay(200);
            turnToColumnSequence(column);
            placeGlyphSequence(column);
            break;
        }
    }
}