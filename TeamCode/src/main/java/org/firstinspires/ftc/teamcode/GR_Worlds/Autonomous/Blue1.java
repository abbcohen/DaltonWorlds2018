package org.firstinspires.ftc.teamcode.GR_Worlds.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "NEW Blue 1", group = "Sensor")
public class Blue1 extends WorldsMasterAuto {

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
            moveForward(.4, 600);
            delay(200);
            strafeRight(.95, 200);
            delay(200);
            turnToColumnSequence(column, 0);
            placeGlyphSequence(column);
            break;
        }
    }
}