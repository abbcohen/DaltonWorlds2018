package org.firstinspires.ftc.teamcode.GR_Worlds.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Blue 2", group = "Sensor")
public class Blue2 extends WorldsMasterAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        declare();
        initGyro();
        waitForStart();
        initVuforia();
        setBaseAngles("blue2");

        waitForStart();

        while (opModeIsActive()) {
            RelicRecoveryVuMark column = getPicto();
            jewelSequence("blue");
            delay(200);
            moveTicksForward(.4,1000);
            delay(950);
            moveTicksRight(.6, 900);
            delay(200);
            turnToColumnSequence(column);
            placeGlyphSequence(column);
            break;
        }
    }
}