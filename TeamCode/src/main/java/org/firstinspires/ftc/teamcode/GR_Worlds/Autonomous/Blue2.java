package org.firstinspires.ftc.teamcode.GR_Worlds.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "NEW Blue 2", group = "Sensor")
public class Blue2 extends WorldsMasterAuto {

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
            jewelSequence("blue");
            delay(200);
            if(column == RelicRecoveryVuMark.CENTER)
                moveForward(.4,1000);
            delay(200);
            turnToColumnSequence(column, 90);
            placeGlyphSequence(column);
            break;
        }
    }
}