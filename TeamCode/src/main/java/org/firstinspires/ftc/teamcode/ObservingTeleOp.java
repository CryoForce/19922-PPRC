package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Observing TeleOp", group="Iterative Opmode")
public class ObservingTeleOp extends OpMode {
    //Declaring outside classes
    HWC bronto;
    BrontoBrain brain;

    private ElapsedTime runTime = new ElapsedTime();

    //int representing the current scoring height
    //bool representing side of robot scoring
    int jctHeight; //0=gnd, 1=low, 2=med, 3=high
    boolean backScoring; //true = delivery from back side, false = delivery from front

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

        jctHeight = 3; //init to high pole
        backScoring = true; //init to back side scoring
        terry.changeScoringSide(backScoring);

        telemetry.addData("Status: ", "Initialized");
    }

    @Override
    public void start() {runTime.reset();}

    @Override
    public void loop() {

        //------------------------------------ GAMEPAD 1 INPUT ----------------------------------//
        if (gamepad1.dpad_up) jctHeight = 3; //high
        else if (gamepad1.dpad_right) jctHeight = 2; //med
        else if (gamepad1.dpad_down) jctHeight = 1; //low
        else if (gamepad1.dpad_left) jctHeight = 0; //gnd

        if (gamepad1.back) {
            backScoring = !backScoring; //toggle backScoring true/false
            terry.changeScoringSide(backScoring);
        }

        if (gamepad1.a) {
            if (gamepad1.x) robotState = Observer.RobotStates.Rest;
            if (gamepad1.y) {

            }
            if (gamepad1.b) robotState = Observer.RobotStates.Drive;
        }
        else if (gamepad1.y) robotState = Observer.RobotStates.Transfer;
        else if (gamepad1.x) robotState = Observer.RobotStates.Intake;
        else if (gamepad1.b) {
            if (jctHeight == 0) robotState = Observer.RobotStates.GndJct;
            if (jctHeight == 1) robotState = Observer.RobotStates.LowJct;
            if (jctHeight == 2) robotState = Observer.RobotStates.MedJct;
            if (jctHeight == 3) robotState = Observer.RobotStates.HighJct;
        }

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            bronto.frontArm.setPower(0);
            bronto.frontElbow.setPower(0);
            bronto.backArm.setPower(0);
            bronto.backElbow.setPower(0);
        }

        //------------------------------------ SETTING CYCLE ------------------------------------//
        if (terry.getRobotState() != Observer.RobotStates.Unknown) {
            terry.setCycleState(robotState);
        } else {
            robotState = Observer.RobotStates.Unknown;
            terry.setCycleState(robotState);
        }

        //------------------------------------ GAMEPAD 2 INPUT ---------------------------------//
        /*
        this is placed AFTER cycle setting since it will override any
        power setting for stuff like intake
         */
        //arm manual increment
        if (gamepad2.left_stick_y != 0) {
            bronto.backArmComponent.incrementTarget(gamepad2.left_stick_y * 70);
        } else if (gamepad2.right_stick_y != 0) {
            bronto.frontArmComponent.incrementTarget(gamepad2.right_stick_y * 70);
        } else if (gamepad2.left_stick_x != 0) {
            bronto.backElbowComponent.incrementTarget(gamepad2.left_stick_x * 15);
        } else if (gamepad2.right_stick_x != 0) {
            bronto.frontElbowComponent.incrementTarget(gamepad2.right_stick_x * 15);
        }
        //servo manual control
        if (gamepad2.left_trigger != 0) { //triggers are forward, check neg
            bronto.backIntakeL.setPower(-gamepad2.left_trigger);
            bronto.backIntakeR.setPower(-gamepad2.left_trigger);
        } else if (gamepad2.right_trigger != 0) {
            bronto.frontIntakeL.setPower(-gamepad2.left_trigger);
            bronto.frontIntakeR.setPower(-gamepad2.left_trigger);
        } else if (gamepad2.left_bumper) {
            bronto.backIntakeL.setPower(1);
            bronto.backIntakeR.setPower(1);
        } else if (gamepad2.right_bumper) {
            bronto.frontIntakeL.setPower(1);
            bronto.frontIntakeR.setPower(1);
        } //TODO: check negatives

        //--------------------------------------- DRIVING --------------------------------------//
        //HWC drive pwr and calculations
        bronto.manualDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //--------------------------------------- TELEMETRY ------------------------------------//
        telemetry.addData("Bronto State: ", robotState);
        telemetry.addData("Terry State: ", terry.getRobotState());
        telemetry.addData("Front Arm State: ", terry.getFrontState());
        telemetry.addData("Back Arm State: ", terry.getBackState());
        telemetry.addData("Back Scoring: ", backScoring);
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