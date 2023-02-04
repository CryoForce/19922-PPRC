package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name="Bronto's TeleOp", group="Iterative Opmode")

public class BrontoTeleOP extends OpMode
{
    /** Declare OpMode members. */
    HWC bronto;
    BrontoBrain brain;

    public enum TeleOpStates {
        RESTING,
        DRIVING,
        INTAKE,
        DELIVERING, //WIPED LOW,MED,HIGH POLE B/C NO DIFFERENCE SINCE NO SENSORS GET TRIPPED
        MOVING,
        TRANSFER,
        UNKNOWN,
        ELBOW_MOVING
    }

    private ElapsedTime runtime = new ElapsedTime();

    boolean manualMode = false;

    int frontArmTarget = 0;
    int backArmTarget = 0;
    int frontElbowTarget = bronto.frontElbowHighPos;
    int backElbowTarget = 0;
    int autoCycle = -1;

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

        backArmTarget = bronto.backArmHighPos; //set back arm to back high pole immediately for power draw issues

        telemetry.addData("Status", "Initialized");

        state = TeleOpStates.MOVING;
        nextState = TeleOpStates.UNKNOWN;

        bronto.drive.setPoseEstimate(bronto.START_POS_TELEOP);
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
        double leftFPower ;
        double rightFPower;
        double leftBPower ;
        double rightBPower;
        double intakePow = 0;
        double outtakePow = 0;

        boolean frontArmIsClose = false;
        boolean backArmIsClose = false;
        boolean frontArmMoving = false;
        boolean backArmMoving = false;

        double drive = -gamepad1.left_stick_y *0.8;
        double turn  =  gamepad1.left_stick_x * 0.6;
        double strafe = -gamepad1.right_stick_x * 0.8;

        //force stop and reset
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.left_stick_button){
            bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

        //force exit cycle
        if (gamepad1.dpad_right && gamepad1.dpad_left){
            autoCycle = -1;
            telemetry.addData("Cycle Status", "CANCELED!");
        }
        if (gamepad1.dpad_down){
            autoCycle = 3;
        }
        if (gamepad1.dpad_up){
            autoCycle--;
        }

        switch (autoCycle) {
            case 3: // Moves to intake
                bronto.drive.followTrajectory(TC.forward(bronto.drive, bronto.START_POS_TELEOP, 5));
                Pose2d newPos2 = TC.forward(bronto.drive, bronto.START_POS_TELEOP, 5).end();
                nextState = TeleOpStates.INTAKE;
                frontElbowTarget = bronto.frontElbowIntakePos;
                backElbowTarget = bronto.backElbowHighPos;
                frontArmTarget = bronto.frontArmIntakePos;
                backArmTarget = bronto.backArmHighPos;
                state = TeleOpStates.MOVING;
                break;
            case 2: // moves to transfer
                nextState = TeleOpStates.TRANSFER;
                frontElbowTarget = bronto.frontElbowTransPos;
                backElbowTarget = bronto.backElbowTransPos;
                frontArmTarget = bronto.frontArmTransPos;
                backArmTarget = bronto.backArmHighPos;
                state = TeleOpStates.MOVING;
                break;
            case 1: // moves to delivery
                bronto.drive.followTrajectory(TC.backwards(bronto.drive, bronto.START_POS_TELEOP,5));
                Pose2d newPos = TC.backwards(bronto.drive, bronto.START_POS_TELEOP,5).end();
                //code to drive to delivery pos
                nextState = TeleOpStates.DELIVERING;
                frontElbowTarget = bronto.frontElbowIntakePos;
                backElbowTarget = bronto.backElbowHighPos;
                frontArmTarget = bronto.frontArmIntakePos;
                backArmTarget = bronto.backArmHighPos;
                state = TeleOpStates.MOVING;
                break;
            case 0:

                autoCycle = 3;
                break;
            default:
                break;

        }

        if (gamepad2.right_trigger != 0) { outtakePow = gamepad2.right_trigger;}
        else if (gamepad2.right_bumper == true) {outtakePow = -1;}
        else {outtakePow = 0;}

        if (gamepad2.left_trigger != 0) { intakePow = gamepad2.left_trigger;}
        else if (gamepad2.left_bumper == true) {intakePow = -1;}
        else {intakePow = 0;}

        /*
        if (gamepad2.right_stick_button){
            manualMode = !manualMode;
        }
         */

        //manual arm control
        if (gamepad2.left_stick_x != 0) { //manual control just changes target, large numbers b/c large ticks needed
            frontElbowTarget += (gamepad2.left_stick_x * 15);
        } else if (gamepad2.left_stick_y != 0) {
            backElbowTarget += (gamepad2.left_stick_y * 15);
        }

        if (gamepad2.right_stick_x != 0) {
            frontArmTarget += (gamepad2.right_stick_x * 70);
        } else if (gamepad2.right_stick_y != 0) {
            backArmTarget += (gamepad2.right_stick_y * 70);
        }

        //calculate drive pwr
        if (drive != 0 || turn != 0) {
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
            /* Strafing */
            leftFPower = -strafe;
            rightFPower = strafe;
            leftBPower = strafe;
            rightBPower = -strafe;
        } else {
            leftFPower = 0;
            rightFPower = 0;
            leftBPower = 0;
            rightBPower = 0;
        }

        if (gamepad1.a && gamepad1.y) {
            autoCycle = -1;
            state = TeleOpStates.MOVING;
            nextState = TeleOpStates.RESTING;
            frontElbowTarget = bronto.frontElbowRestPos;
            backElbowTarget = bronto.backElbowRestPos;
            frontArmTarget = bronto.frontArmRestPos;
            backArmTarget = bronto.backArmRestPos;
        } else if (gamepad1.a && gamepad1.x) {
            autoCycle = -1;
            state = TeleOpStates.MOVING;

        } else if (gamepad1.y) {
            autoCycle = -1;
            state = TeleOpStates.MOVING;
            nextState = TeleOpStates.TRANSFER;
            frontElbowTarget = bronto.frontElbowTransPos;
            backElbowTarget = bronto.backElbowTransPos;
            frontArmTarget = bronto.frontArmTransPos;
            backArmTarget = bronto.backArmHighPos;
        } else if (gamepad1.x) {
            autoCycle = -1;
            state = TeleOpStates.MOVING;
            nextState = TeleOpStates.DELIVERING;
            frontElbowTarget = bronto.frontElbowIntakePos;
            backElbowTarget = bronto.backElbowHighPos;
            frontArmTarget = bronto.frontArmIntakePos;
            backArmTarget = bronto.backArmHighPos;
        } else if (gamepad1.b) { //changing from medium pole to intake cuz apparently thats not here
            autoCycle = -1;
            state = TeleOpStates.MOVING;
            nextState = TeleOpStates.INTAKE;
            frontElbowTarget = bronto.frontElbowIntakePos;
            backElbowTarget = bronto.backElbowHighPos;
            frontArmTarget = bronto.frontArmIntakePos;
            backArmTarget = bronto.backArmHighPos;
        }

        /* else if (gamepad1.a) {
            state = TeleOpStates.MOVING;
            nextState = TeleOpStates.DELIVERING;
            frontElbowTarget = bronto.frontElbowIntakePos;
            backElbowTarget = bronto.backElbowLowPos;
            frontArmTarget = bronto.frontArmIntakePos;
            backArmTarget = bronto.backArmLowPos;
        }
        */

        //setting next state
        switch (state) {
            case RESTING:
                telemetry.addData("Arm State", "Resting");

                //checks if closeEnough AND button pressed to set power to 0 with bool
                if (bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20) && bronto.frontButton.isPressed()) {
                    frontArmIsClose = true;
                }
                if (bronto.backArmComponent.motorCloseEnough(backArmTarget, 20) && bronto.backButton.isPressed()) {
                    backArmIsClose = true;
                }

                break;
            case DRIVING:
                telemetry.addData("Arm State", "Driving");

                //checks if closeEnough AND button pressed to set power to 0 with bool
                if (bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20)) {
                    frontArmIsClose = true;
                }
                if (bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)) {
                    backArmIsClose = true;
                }

                break;
            case DELIVERING:
                telemetry.addData("Arm State", "High Pole");

                telemetry.addData("Distances Array 0", distances[0]);
                telemetry.addData("Distances Array 1", distances[1]);
                telemetry.addData("Distances Array 2", distances[2]);
                //really stupid way to do a running average but i dont care have fun!
                //also TODO: make this a method or class
                double distAvg = bronto.backHighDist;
                if (distances[2] != 0) {
                    distances [2] = distances [1]; //pushback old values
                    distances [1] = distances [0];
                    distances [0] = bronto.backDistanceSensor.getDistance(DistanceUnit.CM); //new value
                    distAvg = (distances[0] + distances[1] + distances[2]) / 3;
                } else if (distances[0] == 0) { //dummy easy way to add values at beginning
                    distances [0] = bronto.backDistanceSensor.getDistance(DistanceUnit.CM);
                } else if (distances[1] == 0) {
                    distances [1] = bronto.backDistanceSensor.getDistance(DistanceUnit.CM);
                } else {
                    distances [2] = bronto.backDistanceSensor.getDistance(DistanceUnit.CM);
                }
                telemetry.addData("Distances Avg", distAvg);
                //TODO: check if should be -= and adjust target
                backElbowTarget += bronto.moveBySetDistance(distAvg, bronto.backHighDist);

                //checks if closeEnough AND button pressed to set power to 0 with bool
                //if only target close enough and buttong NOT pressed, increments target until buttons pressed
                if (bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20)) {
                    frontArmIsClose = true;
                }
                if (bronto.backArmComponent.motorCloseEnough(backArmTarget, 20) && bronto.backButton.isPressed()) {
                    backArmIsClose = true;
                } else if (bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)) {
                    backArmTarget -= 20; //if motor is reversed, this must be +=
                }

                if (bronto.returnColor(bronto.backIntakeSensor) == "unknown" && bronto.backIntakeL.getPower() !=0 || bronto.backIntakeR.getPower() !=0) {
                    outtakePow = 0;
                    autoCycle--;
                    state = TeleOpStates.UNKNOWN;
                }
                else{
                //checks if front/back arm are close enough and if distance is close enough
                if (backArmIsClose && bronto.closeEnough((int) distAvg, bronto.backHighDist, 1)) {
                    outtakePow = -1;

                }}

                break;
            case INTAKE:
                telemetry.addData("Arm State", "Intake");
                        /*
                        basically this should add to the target some small value once it is intaking to fine tune its
                        distance from the ground
                         */
                frontElbowTarget -= bronto.moveByDistance(bronto.frontDistanceSensor, bronto.frontIntakeDist);

                telemetry.addData ("Front Color: ", bronto.returnColor(bronto.frontIntakeSensor));
                if (bronto.returnColor(bronto.frontIntakeSensor) != "unknown" && bronto.frontIntakeL.getPower() != 0 || bronto.frontIntakeR.getPower() != 0){
                    intakePow = 0;
                    autoCycle--;
                    state = TeleOpStates.UNKNOWN;
                }
                else {intakePow = -1;}

                //checks if closeEnough AND button pressed to set power to 0 with bool
                if (bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20) && bronto.frontButton.isPressed()) {
                    frontArmIsClose = true;
                }
                if (bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)) {
                    backArmIsClose = true;
                }

                break;
            case MOVING:
                telemetry.addData("Arm State", "Moving");
                if (nextState == TeleOpStates.DELIVERING){
                    frontArmMoving = true;
                    backArmMoving = true;
                }
                else if (nextState == TeleOpStates.TRANSFER){
                    frontArmMoving = true;
                    backArmMoving = true;
                }
                else if (nextState == TeleOpStates.INTAKE){
                    frontArmMoving = true;
                    backArmMoving = true;
                }
                if (bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 50) &&
                        bronto.backArmComponent.motorCloseEnough(backArmTarget, 50)) { //putting range high because it will keep moving
                    state = TeleOpStates.ELBOW_MOVING;
                    nextState = TeleOpStates.UNKNOWN;
                }

                break;
            case ELBOW_MOVING:
                telemetry.addData("Arm State", "Elbows moving");
                if (bronto.frontElbowComponent.motorCloseEnough(frontElbowTarget, 20) &&
                bronto.backElbowComponent.motorCloseEnough(backElbowTarget, 20)){
                    state = nextState;
                    nextState = TeleOpStates.UNKNOWN;
                }

            case TRANSFER:
                telemetry.addData("Arm State", "Transfer");
                //TODO: check if should be -= and adjust target
                //thus both should be at the same height and aligned
                //frontElbowTarget += bronto.moveByDistance(bronto.frontDistanceSensor, 50);
                //backElbowTarget += bronto.moveByDistance(bronto.backDistanceSensor, 50);
                //TODO: if the above works well, we should porbably incorporate as an && for the transfer position detection rather than a constant loop
                /*
                //commenting out until transfer sensor is connected
                if (bronto.returnColor(bronto.transferSensor) != "unknown"){
                    intakePow = 1;}
                 */

                //if motor close enough but button not pressed increment both arms
                if (bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20) && bronto.frontButton.isPressed()) {
                    frontArmIsClose = true;
                } else if (bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20)) {
                    frontArmTarget += 20; //if motor is reversed, this must be -=
                }
                if (bronto.backArmComponent.motorCloseEnough(backArmTarget, 20) && bronto.backButton.isPressed()) {
                    backArmIsClose = true;
                } else if (bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)) {
                    backArmTarget -= 20; //if motor is reversed, this must be +=
                }

                //checks if front/back arm are close enough to begin transfer
                if (bronto.returnColor(bronto.backIntakeSensor) != "unknown" && bronto.returnColor(bronto.frontIntakeSensor) == "unknown" && bronto.frontIntakeL.getPower() != 0) {
                    intakePow = 0;
                    autoCycle--;
                    state = TeleOpStates.UNKNOWN;
                }  else{
                if (frontArmIsClose && backArmIsClose) {
                    intakePow = -1;
                    outtakePow = -1;


                }}

                break;
            case UNKNOWN:
                telemetry.addData("Arm State", "Unknown");
                //checks if closeEnough to set power to 0 with bool
                if (bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20)) {
                    frontArmIsClose = true;
                }
                if (bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)) {
                    backArmIsClose = true;
                }

                break;
            default:
                state = TeleOpStates.UNKNOWN;
                telemetry.addData("Arm Position", "Unknown");

        }

        bronto.leftFront.setPower(leftFPower);
        bronto.leftRear.setPower(leftBPower);
        bronto.rightFront.setPower(rightFPower);
        bronto.rightRear.setPower(rightBPower);
        bronto.frontIntakeL.setPower(intakePow);
        bronto.frontIntakeR.setPower(intakePow);
        bronto.backIntakeL.setPower(outtakePow);
        bronto.backIntakeR.setPower(outtakePow);

        if (frontArmTarget > 5500) {
            frontArmTarget = 5500;
        } else if (frontArmTarget < 0) {
            frontArmTarget = 0;
        }
        if (backArmTarget < -6200) {
            backArmTarget = -6200;
        } else if (backArmTarget > 0) {
            backArmTarget = 0;
        }

        //after going through every state to determine what is close for the arms, set to 0 if they are close enough
        if (frontArmIsClose) {bronto.frontArm.setPower(0);}
        else {bronto.frontArmComponent.moveUsingPID(frontArmTarget);}
        if (backArmIsClose) {bronto.backArm.setPower(0);}
        else {bronto.backArmComponent.moveUsingPID(backArmTarget);}

        if (state == TeleOpStates.MOVING || state == TeleOpStates.RESTING){bronto.frontElbow.setPower(0); bronto.backElbow.setPower(0);}
        else {bronto.frontElbowComponent.moveUsingPID(frontElbowTarget); bronto.backElbowComponent.moveUsingPID(backElbowTarget);}


        

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
        telemetry.addLine();
        telemetry.addData("Auto Cycle", autoCycle);
        //telemetry.addData("Positions", "front Arm %d, Back Arm %d, Front Elbow %d, Back Elbow %d", bronto.frontArm.getCurrentPosition(), bronto.backArm.getCurrentPosition(), bronto.frontElbow.getCurrentPosition(), bronto.backElbow.getCurrentPosition());
        // telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", bronto.leftFront, bronto.rightFront, bronto.leftRear, bronto.rightRear, bronto.frontArm.getPower(), bronto.frontElbow.getPower());
        telemetry.update();
    }

    /** Code to run ONCE after the driver hits STOP. */
    @Override
    public void stop () {
    }
}