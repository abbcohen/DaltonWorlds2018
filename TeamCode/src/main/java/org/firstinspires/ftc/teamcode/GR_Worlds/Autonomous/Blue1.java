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
        setBaseAngles("blue1");

        waitForStart();

        while (opModeIsActive()) {
            RelicRecoveryVuMark column = getPicto();
            jewelSequence("blue");
            sleep(200);
            // TODO from abby: 4/12/18 test these numbers:
//            moveTicksForward(.4, 1900); //this shit works

                                          //this shit may not work
            moveTicksForward(.4, 800);
            sleep(200);
            //TURN TO THE CORRECT COLUMN
            //turnToColumnSequence(column);
            placeGlyphSequence(column);
            break;
        }
    }
}