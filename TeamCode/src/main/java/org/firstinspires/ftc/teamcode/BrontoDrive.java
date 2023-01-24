package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name="Bronto's Driving Test", group="Iterative Opmode")

public class BrontoDrive extends OpMode
{
    /** Declare OpMode members. */
    HWC bronto;
    BrontoBrain brain;

    public enum TeleOpStates {
        RESTING,
        INTAKE,
        DELIVERING, //WIPED LOW,MED,HIGH POLE B/C NO DIFFERENCE SINCE NO SENSORS GET TRIPPED
        MOVING,
        TRANSFER,
        UNKNOWN
    }

    private ElapsedTime runtime = new ElapsedTime();

    boolean manualMode = false;

    int frontArmTarget = 0;
    int backArmTarget = 0;
    int frontElbowTarget = 0;
    int backElbowTarget = 0;

    double [] distances = new double [3];

    TeleOpStates state = TeleOpStates.RESTING;
    TeleOpStates nextState = TeleOpStates.UNKNOWN; //determines when a moving state completes and will go to

    @Override
    public void init() {
        bronto = new HWC(hardwareMap, telemetry);
        brain = new BrontoBrain(bronto);
        telemetry.addData("Status", "Initializing");
        bronto.frontElbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontElbow.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bronto.leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        bronto.leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        bronto.rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        bronto.rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        //backArmTarget = bronto.backArmHighPos; //set back arm to back high pole immediately for power draw issues

        telemetry.addData("Status", "Initialized");

        state = TeleOpStates.MOVING;
        nextState = TeleOpStates.UNKNOWN;
    }



    /** Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY. */
    @Override
    public void init_loop() {
    }

    /** Code to run ONCE when the driver hits PLAY. */
    @Override
    public void start(){
        runtime.reset();
    }

    @Override
    public void loop() {
        /* Setup a variable for each drive wheel to save power level for telemetry. */

        double drive = -gamepad1.left_stick_y *0.8;
        double turn  =  gamepad1.left_stick_x * 0.6;
        double strafe = -gamepad1.right_stick_x * 0.8;

        bronto.leftFront.setPower(0);
        bronto.rightFront.setPower(0);
        bronto.leftRear.setPower(0);
        bronto.rightRear.setPower(0);
        if (gamepad1.y) {
            bronto.leftFront.setPower(.2);
        }
        if (gamepad1.x) {
            bronto.leftRear.setPower(.2);
        }
        if (gamepad1.b) {
            bronto.rightFront.setPower(.2);
        }
        if (gamepad1.a) {
            bronto.rightRear.setPower(.2);
        }

        telemetry.addLine();
        telemetry.addData("frontArm Target", frontArmTarget);
        telemetry.addData("frontArm Pos", bronto.frontArm.getCurrentPosition());
        telemetry.addData("frontArm Pwr", bronto.frontArm.getPower());
        telemetry.addLine();
        telemetry.addData("frontElbow Target", frontElbowTarget);
        telemetry.addData("frontElbow Pos", bronto.frontElbow.getCurrentPosition());
        telemetry.addData("frontElbow Pwr", bronto.frontElbow.getPower());
        telemetry.addLine();
        telemetry.addData("backArm Target", backArmTarget);
        telemetry.addData("backArm Pos", bronto.backArm.getCurrentPosition());
        telemetry.addData("backArm Pwr", bronto.backArm.getPower());
        telemetry.addLine();
        telemetry.addData("backElbow Target", backElbowTarget);
        telemetry.addData("backElbow Pos", bronto.backElbow.getCurrentPosition());
        telemetry.addData("backElbow Pwr", bronto.backElbow.getPower());
        telemetry.addLine();
        telemetry.addData ("front Distance", bronto.frontDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData ("back Distance", bronto.backDistanceSensor.getDistance(DistanceUnit.CM));
        //telemetry.addData("Positions", "front Arm %d, Back Arm %d, Front Elbow %d, Back Elbow %d", bronto.frontArm.getCurrentPosition(), bronto.backArm.getCurrentPosition(), bronto.frontElbow.getCurrentPosition(), bronto.backElbow.getCurrentPosition());
        // telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", bronto.leftFront, bronto.rightFront, bronto.leftRear, bronto.rightRear, bronto.frontArm.getPower(), bronto.frontElbow.getPower());
        telemetry.update();
    }

    /** Code to run ONCE after the driver hits STOP. */
    @Override
    public void stop () {
    }
}