package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Testing Op", group="Iterative Opmode")

public class testingOp extends OpMode
{
    /** Declare OpMode members. */
    HWC bronto;
    private ElapsedTime runtime = new ElapsedTime();
    String colorFront;
    String colorBack;

    @Override
    public void init() {
        bronto = new HWC(hardwareMap, telemetry);


        telemetry.addData("Status", "Initializing");
        bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontElbow.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bronto.frontArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bronto.backElbow.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bronto.backArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bronto.backIntakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        bronto.frontIntakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }



    /** Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY. */
    @Override
    public void init_loop() {
    }

    /** Code to run ONCE when the driver hits PLAY. */
    @Override
    public void start(){
        runtime.reset();
        ;
    }
    @Override
    public void loop() {
    bronto.frontArm.setPower(-gamepad1.left_stick_y);
    bronto.frontElbow.setPower(-gamepad1.right_stick_y);
    bronto.backArm.setPower(-gamepad2.left_stick_y);
    bronto.backElbow.setPower(-gamepad2.right_stick_y);
    if (gamepad1.a){
         colorFront = bronto.returnColor(bronto.frontIntakeSensor);
         colorBack = bronto.returnColor(bronto.backIntakeSensor);
    }
    else  {colorFront = " "; colorBack = "";}

    /*bronto.frontIntakeL.setPower(gamepad2.left_stick_y);
    bronto.frontIntakeR.setPower(gamepad2.left_stick_y);
    bronto.backIntakeL.setPower(gamepad2.left_stick_y);
    bronto.backIntakeR.setPower(gamepad2.left_stick_y);

    */

        telemetry.addData("frontArm", bronto.frontArm.getCurrentPosition());
        telemetry.addData("frontElbow", bronto.frontElbow.getCurrentPosition());
        telemetry.addData("backArm", bronto.backArm.getCurrentPosition());
        telemetry.addData("backElbow", bronto.backElbow.getCurrentPosition());
        telemetry.addData ("front Distance", bronto.frontDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData ("back Distance", bronto.backDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Front color", colorFront);
        telemetry.addData("Back color", colorBack);
        telemetry.addData("Front Button", bronto.frontButton.isPressed());
        telemetry.addData("Back Button", bronto.backButton.isPressed());
       telemetry.addData("Arms", "front Arm , front elbow " ,bronto.frontArm.getCurrentPosition(), bronto.frontElbow.getCurrentPosition());
        telemetry.update();
    }
}