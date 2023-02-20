package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Observing TeleOp", group="Iterative Opmode")
public class ObservingTeleOp extends OpMode {
    //Declaring outside classes
    HWC bronto;
    BrontoBrain brain;

    private ElapsedTime runTime = new ElapsedTime();

    //initialize drive motor powers
    double leftFPwr;
    double rightFPwr;
    double leftBPwr;
    double rightBPwr;

    //initialize motor vals
    double drive;
    double turn;
    double strafe;

    //initialize robotState and observer
    Observer terry;
    Observer.RobotStates robotState;

    @Override
    public void init() {
        bronto = new HWC(hardwareMap, telemetry);
        terry = new Observer(bronto);
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

        //set arm motor directions
        bronto.frontArm.setDirection(DcMotorSimple.Direction.FORWARD);
        bronto.frontElbow.setDirection(DcMotorSimple.Direction.FORWARD);
        bronto.backArm.setDirection(DcMotorSimple.Direction.REVERSE);
        bronto.backElbow.setDirection(DcMotorSimple.Direction.FORWARD);

        robotState = Observer.RobotStates.Rest;

        telemetry.addData("Status: ", "Initialized");
    }

    @Override
    public void start() {runTime.reset();}

    @Override
    public void loop() {
        //robot too fast and Jack bad at driving
        drive = -gamepad1.left_stick_y * .8;
        turn = gamepad1.left_stick_x * .6;
        strafe = -gamepad1.right_stick_x * .8;

        //calculate drive pwr
        if (drive != 0 || turn != 0) {
            leftFPwr = Range.clip(drive + turn, -1.0, 1.0);
            rightFPwr = Range.clip(drive - turn, -1.0, 1.0);
            leftBPwr = Range.clip(drive + turn, -1.0, 1.0);
            rightBPwr = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
            /* Strafing */
            leftFPwr = -strafe;
            rightFPwr = strafe;
            leftBPwr = strafe;
            rightBPwr = -strafe;
        } else {
            leftFPwr = 0;
            rightFPwr = 0;
            leftBPwr = 0;
            rightBPwr = 0;
        }

        if (gamepad1.y) robotState = Observer.RobotStates.Intake;
        if (gamepad1.x) robotState = Observer.RobotStates.Transfer;
        if (gamepad1.b) robotState = Observer.RobotStates.HighJct; //eventually needs to be modifiable to diff poles

        bronto.leftFront.setPower(leftFPwr);
        bronto.leftRear.setPower(leftBPwr);
        bronto.rightFront.setPower(rightFPwr);
        bronto.rightRear.setPower(rightBPwr);

        if (terry.getRobotState() != Observer.RobotStates.Unknown) {
            terry.setCycleState(robotState);
        } else {
            robotState = Observer.RobotStates.Unknown;
            terry.setCycleState(robotState);
        }

        telemetry.addLine();
        telemetry.addData("frontArm Target", bronto.frontArmComponent.getTarget());
        telemetry.addData("frontArm Pos", bronto.frontArm.getCurrentPosition());
        telemetry.addData("frontArm Pwr", bronto.frontArm.getPower());
        telemetry.addLine();
        telemetry.addData("frontElbow Target", bronto.frontElbowComponent.getTarget());
        telemetry.addData("frontElbow Pos", bronto.frontElbow.getCurrentPosition());
        telemetry.addData("frontElbow Pwr", bronto.frontElbow.getPower());
        telemetry.addLine();
        telemetry.addData("backArm Target", bronto.backArmComponent.getTarget());
        telemetry.addData("backArm Pos", bronto.backArm.getCurrentPosition());
        telemetry.addData("backArm Pwr", bronto.backArm.getPower());
        telemetry.addLine();
        telemetry.addData("backElbow Target", bronto.backElbowComponent.getTarget());
        telemetry.addData("backElbow Pos", bronto.backElbow.getCurrentPosition());
        telemetry.addData("backElbow Pwr", bronto.backElbow.getPower());
        telemetry.addLine();
        telemetry.addData ("front Distance", bronto.frontDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData ("back Distance", bronto.backDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addLine();
        //telemetry.addData("Positions", "front Arm %d, Back Arm %d, Front Elbow %d, Back Elbow %d", bronto.frontArm.getCurrentPosition(), bronto.backArm.getCurrentPosition(), bronto.frontElbow.getCurrentPosition(), bronto.backElbow.getCurrentPosition());
        // telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", bronto.leftFront, bronto.rightFront, bronto.leftRear, bronto.rightRear, bronto.frontArm.getPower(), bronto.frontElbow.getPower());
        telemetry.update();

    }

}