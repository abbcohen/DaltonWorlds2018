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
        //setStartAngle();

        waitForStart();

        while (opModeIsActive()) {
            RelicRecoveryVuMark column = getPicto();
            //jewelSequence("blue");
            //delay(200);
            // TODO from abby: 4/12/18 test these numbers:
            moveTicksForward(.4, 2000);
            //delay(200);
            // TODO from abby: 4/12/18 test these numbers:
            strafeRight(.4, 350);
            //delay(200);
            //turnToColumnSequence(column, 90);
            placeGlyphSequence(column);
            break;
        }
    }
}