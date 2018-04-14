package org.firstinspires.ftc.teamcode.GR_Worlds.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="massive legend jack", group="Iterative Opmode")
public class Jack_legend extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;
    private DcMotor Pulley = null;
    private DcMotor NomLeft = null;
    private DcMotor NomRight = null;
    private DcMotor relicMotor = null;
    private Servo Servo1 = null;
    private Servo jewelTwister = null;
    private Servo backPlate = null;
    //private Servo Servo2 = null;
    //private Servo ServoRelicRelease = null;
    private Servo handServo = null;
    private Servo elbowServo = null;
    //private DcMotor ThiccBoiRetriever= null;
    private Servo colorServo = null;
    double startTime = runtime.milliseconds();
    double relicPosition = .5;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Pulley = hardwareMap.get(DcMotor.class, "Pulley");
        NomLeft = hardwareMap.get(DcMotor.class, "NomLeft");
        NomRight = hardwareMap.get(DcMotor.class, "NomRight");
        relicMotor = hardwareMap.get(DcMotor.class, "relicMotor");
        jewelTwister = hardwareMap.get(Servo.class, "jewelTwister");
        backPlate = hardwareMap.get(Servo.class, "backPlate");
        //ThiccBoiRetriever = hardwareMap.get(DcMotor.class, "ThiccBoiRetriever");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        //Servo2 = hardwareMap.get(Servo.class, "Servo2");
        //ServoRelicRelease = hardwareMap.get(Servo.class, "ServoRelicRelease");
        handServo = hardwareMap.get(Servo.class, "handServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        colorServo = hardwareMap.get(Servo.class, "colorServo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        Pulley.setDirection(DcMotor.Direction.FORWARD);
        NomLeft.setDirection(DcMotor.Direction.FORWARD);
        NomRight.setDirection(DcMotor.Direction.REVERSE);
        relicMotor.setDirection(DcMotor.Direction.FORWARD);
        //ThiccBoiRetriever.setDirection(DcMotor.Direction.FORWARD);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ThiccBoiPlacer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
        Servo1.setPosition(0.3);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public void loop() {
        double threshold = 0.5;
        double slowNumber = 0.95;

        if (gamepad1.left_stick_x < -threshold || gamepad1.left_stick_x > threshold) {
            if(gamepad1.left_bumper){
                FrontLeft.setPower(gamepad1.left_stick_x/2);
                FrontRight.setPower(-gamepad1.left_stick_x/2);
                BackLeft.setPower(-gamepad1.left_stick_x/2);
                BackRight.setPower(gamepad1.left_stick_x/2);
            }
            else{
                FrontLeft.setPower(gamepad1.left_stick_x);
                FrontRight.setPower(-gamepad1.left_stick_x);
                BackLeft.setPower(-gamepad1.left_stick_x);
                BackRight.setPower(gamepad1.left_stick_x);
            }

        } else if (gamepad1.left_stick_y < -threshold || gamepad1.left_stick_y > threshold || gamepad1.right_stick_x < -threshold || gamepad1.right_stick_x > threshold) {
            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            //make sure left and right power are outside thres
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            if (leftPower > threshold || leftPower < -threshold || rightPower < -threshold || rightPower > threshold) {
                if(gamepad1.left_bumper){
                    FrontLeft.setPower(leftPower/2);
                    BackLeft.setPower(leftPower/2);
                    FrontRight.setPower(rightPower/2);
                    BackRight.setPower(rightPower/2);
                }else{
                    FrontLeft.setPower(leftPower*slowNumber);
                    BackLeft.setPower(leftPower*slowNumber);
                    FrontRight.setPower(rightPower*slowNumber);
                    BackRight.setPower(rightPower*slowNumber);
                }

            } else {
                FrontRight.setPower(0);
                FrontLeft.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
                //   double leftPower;
                //   double rightPower;
            }


        }
        // else if (gamepad1.right_stick_y < -threshold || gamepad1.right_stick_y> threshold) {
        //     FrontLeft.setPower(-gamepad1.right_stick_x);
        //    FrontRight.setPower(-gamepad1.right_stick_x);
        //    BackLeft.setPower(-gamepad1.right_stick_x);
        // }
        else {
            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            //   double leftPower;
            //   double rightPower;
        }

        //nom motors
        double nomPower = .95;
        if (gamepad1.right_trigger > .2) {
            NomRight.setPower(-nomPower);
            NomLeft.setPower(-nomPower);
        }
        else if(gamepad1.left_trigger >.2){
            NomRight.setPower(0);
            NomLeft.setPower(0);
        }
        else{
            NomRight.setPower(nomPower);
            NomLeft.setPower(nomPower);
        }


        //Servo Stuff


        if (gamepad2.b) {
            Servo1.setPosition(0.18);
            //Servo2.setPosition(0.3);
        } else if (gamepad2.x) {
            Servo1.setPosition(0.30);
            //Servo2.setPosition(0.6);
        } else if (gamepad2.y) {
            Servo1.setPosition(0.5);
            //Servo2.setPosition(0.7);
        } else if (gamepad2.a) {
            Servo1.setPosition(0.25);
            //Servo2.setPosition(0.7);
        }else{

        }


        //back plate
        if(gamepad1.a){
            backPlate.setPosition(.9);
        }
        else{
            backPlate.setPosition(.3);
        }

        //lift

        if (gamepad2.dpad_up) {
            Pulley.setPower(-.95);
        } else if (gamepad2.dpad_down) {
            Pulley.setPower(.95);
        } else {
            Pulley.setPower(0);
        }

        //relic placer

        if(gamepad2.right_stick_y > .4){
            relicMotor.setPower(-.6);
        }else if(gamepad2.right_stick_y < -.4){
            relicMotor.setPower(.6);
        }
        else{
            relicMotor.setPower(0);
        }

        //relic elbow
        if(gamepad2.right_bumper){
            elbowServo.setPosition(.90);
        }
        else if(gamepad2.left_bumper) {
            elbowServo.setPosition(.5);
        }

        //relic hand
        if(gamepad2.right_trigger>.4){
            handServo.setPosition(.9);
        }
        else if(gamepad2.left_trigger>.4){
            handServo.setPosition(.45);
        }

        telemetry.addData("handServo", handServo.getPosition());
        telemetry.addData("elbowServo", elbowServo.getPosition());
        //jewel servo
        colorServo.setPosition(0.9);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    /*
     * Code to run ONCE after the driver hits STOP
     */
    }
}