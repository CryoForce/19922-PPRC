package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Revised TeleOp", group="Iterative Opmode")
public class revisedTeleOp extends OpMode {
    //Declaring outside classes
    HWC bronto;
    BrontoBrain brain;

    public enum FrontStates {
        MTR,
        MTD,
        MTI,
        MTT,
        MTH,
        MTM,
        MTL,
        MTG,
        Rest,
        Drive,
        Intake,
        Transfer,
        Delivery,
        Unknown
    }
    public enum BackStates {
        MTR,
        MTD,
        MTI,
        MTT,
        MTH,
        MTM,
        MTL,
        MTG,
        Rest,
        Drive,
        Intake,
        Transfer,
        Delivery,
        Unknown
    }

    private ElapsedTime runTime = new ElapsedTime();

    //initialize arm targets
    int frontArmTarget = 0;
    int backArmTarget = 0;
    int frontElbowTarget = 0;
    int backElbowTarget = 0;

    //initialize motor powers
    double leftFPwr;
    double rightFPwr;
    double leftBPwr;
    double rightBPwr;

    //initialize motor vals
    double drive;
    double turn;
    double strafe;

    //initialize servo powwers
    double intakePwr = 0;
    double outtakePwr = 0;

    //initialzie Arm values are close and elbow is on
    boolean frontArmIsClose = false;
    boolean backArmIsClose = false;
    boolean frontElbowOn = false;
    boolean backElbowOn = false;

    double [] frontDistances = new double [3];
    double [] backDistances = new double [3];

    FrontStates frontState = FrontStates.Rest;
    BackStates backState = BackStates.Rest;

    @Override
    public void init() {
        bronto = new HWC(hardwareMap, telemetry);
        brain = new BrontoBrain(bronto);
        telemetry.addData("Status: ", "Initializing");
        //stop and reset and set to run w/o encoder
        bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set driving motor directions
        bronto.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        bronto.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        bronto.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        bronto.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        frontState = FrontStates.Rest;
        backState = BackStates.Rest;

        telemetry.addData("Status: ", "Initialized");
    }

    @Override
    public void start() {runTime.reset();}

    @Override
    public void loop() {
        //robot too fast and Omar bad at driving
        drive = -gamepad1.left_stick_y *.8;
        turn = gamepad1.left_stick_x * .6;
        strafe = -gamepad1.right_stick_x * .8;

        //turn on elbows, only turned off in certain states
        frontElbowOn = true;
        backElbowOn = false;

        switch (frontState) {
            case MTI:
                telemetry.addData("Front Arm State: ", "MTI");
                if (frontArmTarget < Math.abs(bronto.frontArmComponent.armTicksUsingAngle(0))) {
                    frontElbowOn = true;
                    if (frontArmTarget < Math.abs(bronto.frontArmComponent.armTicksUsingAngle(-30))
                            && !bronto.frontElbowComponent.motorCloseEnough(frontElbowTarget, 100)) {
                        frontArmIsClose = true;
                    } else {
                        frontArmIsClose = false;
                    }
                }
            case Rest:
                telemetry.addData("Front Arm State: ", "Rest");
                break;
            case Drive:
                telemetry.addData("Front Arm State: ", "Drive");
                break;
            case Intake:
                break;
        }


        switch (backState) {
            case MTI:
                break;

            case MTR:
                break;

            case MTD:
                break;

            case MTG:
                break;

            case MTH:
                break;

            case MTL:
                break;

            case MTM:
                break;

            case MTT:
                break;

            case Rest:
                break;

            case Drive:
                break;

            case Transfer:
                break;

            case Delivery:
                break;

            case Unknown:
                break;

            default:
                backState = BackStates.Unknown;
                break;




        }

    }




}
