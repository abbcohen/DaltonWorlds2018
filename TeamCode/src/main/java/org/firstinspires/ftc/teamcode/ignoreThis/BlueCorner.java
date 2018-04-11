package org.firstinspires.ftc.teamcode.ignoreThis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "BlueSide")
public class BlueCorner extends AutoBase {
    RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        CBL = hardwareMap.get(ColorSensor.class, "CBL");
        CBOT = hardwareMap.get(ColorSensor.class, "CBOT");
        JS = hardwareMap.get(Servo.class, "JS");
        lift = hardwareMap.get(DcMotor.class, "lift");
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        rightBottom = hardwareMap.get(CRServo.class, "rightBottom");
        leftBottom = hardwareMap.get(CRServo.class, "leftBottom");
        rightTop = hardwareMap.get(CRServo.class, "rightTop");
        leftTop = hardwareMap.get(CRServo.class, "leftTop");

        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        lift.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        initGyro();
        initVuforia();

//        startingAngle = imu.getAngularOrientation().firstAngle; //grabbers facing away from wall
//        telemetry.addData("start", startingAngle);
        telemetry.update();
        strafe(false);
        JS.setPosition(JEWEL_SERVO_UP);
        liftServo.setPosition(LIFT_FLIPDOWN);
        waitForStart();
        if (isStopRequested()) return;
        if (!isStopRequested() && opModeIsActive())  column = getPicto();
        if (!isStopRequested() && opModeIsActive()) intake(.81);
        if (!isStopRequested() && opModeIsActive()) lift(.7, 500);
        if (!isStopRequested() && opModeIsActive()) stopIntake();
        if (!isStopRequested() && opModeIsActive()) pushJewel();
        if (!isStopRequested() && opModeIsActive()) delay(500);
        if (!isStopRequested() && opModeIsActive()) drive(0,-.3,0,1350);
        if (!isStopRequested() && opModeIsActive()) delay(500);
        if (!isStopRequested() && opModeIsActive()) drive(0,0,-.3,350);
        if (!isStopRequested() && opModeIsActive()) delay(500);
        if (!isStopRequested() && opModeIsActive()) driveUntilColorBlue(-.3);
        if (!isStopRequested() && opModeIsActive()) delay(500);
        if (!isStopRequested() && opModeIsActive()) {
            if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                if (!isStopRequested() && opModeIsActive()) turn(-.3, 170);
            } else if (column == RelicRecoveryVuMark.LEFT) {
                if (!isStopRequested() && opModeIsActive()) turn(.3, 155);
            } else if (column == RelicRecoveryVuMark.RIGHT) {
                if (!isStopRequested() && opModeIsActive()) turn(-.3, 136.5);
            }
        }
        if (!isStopRequested() && opModeIsActive()) delay(500);
        if (!isStopRequested() && opModeIsActive()) lift(-.7,400);
        if (!isStopRequested() && opModeIsActive()) outtake(.81);
        if (!isStopRequested() && opModeIsActive()) delay(1000);
        if (!isStopRequested() && opModeIsActive()) stopIntake();
        if (!isStopRequested() && opModeIsActive()) delay(500);
        if (!isStopRequested() && opModeIsActive()) drive(0, 0, .3, 1800);
        if (!isStopRequested() && opModeIsActive()) delay(500);
        if (!isStopRequested() && opModeIsActive()) drive(0, 0, -.3, 350);
    }
/*
    public void run(int state) {
        if (state == 0) {
            closeGrabber();
            delay(500);
            lift.setPower(.4);
            delay(500);
            lift.setPower(0);
        } else if (state == 1) {
            column = getPicto();
            telemetry.addData("column", column);
            telemetry.update();
            delay(500);
        } else if (state == 2) {
            pushJewel();
        } else if (state == 3) {
            if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                drive(0, -.3, 0, 1500);
            } else if (column == RelicRecoveryVuMark.LEFT) {
                drive(0, -.3, 0, 1400);
            } else if (column == RelicRecoveryVuMark.RIGHT) {
                drive(0, -.3, 0, 1700);
            }
        } else if (state == 4) {
            if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                turn(-.2, 154);
            } else if (column == RelicRecoveryVuMark.LEFT) {
                turn(-.2, 175);
            } else if (column == RelicRecoveryVuMark.RIGHT) {
                turn(-.2, 147);
            }
        } else if (state == 5) {
            lift.setPower(-.4);
            delay(600);
            lift.setPower(0);
            delay(500);
            openGrabberFlat();
            delay(800);
        } else if (state == 6) {
            drive(0, 0, .3, 1800);
        } else {
            delay(1000);
            drive(0, 0, -.3, 200);
        }
    }
    */
}