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
            sleep(200);
            moveTicksForward(.4,1200);
            sleep(950);
            moveTicksRight(.6, 800);
            sleep(200);
            turnToColumnSequence(column);
            placeGlyphSequence(column);
            break;
        }
    }
}