package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class HWC {
    // Declare empty variables for robot hardware
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, frontArm, frontElbow, backArm, backElbow;
    public CRServo frontIntakeL, frontIntakeR, backIntakeR, backIntakeL;
    public ColorSensor frontIntakeSensor, backIntakeSensor, transferSensor;
    public DistanceSensor frontDistanceSensor, backDistanceSensor;
    public int cameraMonitorViewId;
    public TouchSensor frontButton, backButton;

    // CV vars
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    SleeveDetection sleeveDetection = new SleeveDetection(145,168,30,50);

    // Declare other variables to be used here
    Telemetry telemetry;
    ElapsedTime time = new ElapsedTime();

    // Other auton variables
    public int cycleCount = 0;
    public int parkingZone = 0;

    // Roadrunner start positions
    public final Pose2d START_POS_RIGHT = new Pose2d(35, -60, Math.toRadians(90));
    public final Pose2d START_POS_LEFT = new Pose2d(35, -60, Math.toRadians(90));
    public final Pose2d START_POS_TELEOP = new Pose2d(0, -60, Math.toRadians(90));
    
    // Roadrunner drive
    public SampleMecanumDrive drive;

    // Code variables
    public static final double ONE_CM_IN_PPR = 7.9;
    public static final double ONE_DEGREE_IN_PPR = 4.27;

    // autonStates Enum
    public enum autonStates {
        SCANNING_FOR_SIGNAL,
        MOVING_TO_POLE,
        DELIVERING_CONE,
        MOVING_TO_STACK,
        PICKING_UP_CONE,
        PARKING_NO_VALUE,
        PARKING_VALUE
    }

    public enum START_POSITION {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    // armPositions Enum
    public enum armPositions {
        RESTING,
        INTAKE,
        LOW_POLE,
        MED_POLE,
        HIGH_POLE,
        TRANSFER,
        UNKNOWN
    }

    //We should both be using these in all our code. Makes it much easier to tune as only one person has to
    int frontArmRestPos = 0;
    int frontArmDrivePos = 728;
    int frontArmIntakePos = 2304; //changed back from 0
    int frontArmGndPos = frontArmIntakePos;
    int frontArmLowPos = frontArmIntakePos; //optimize moving time
    int frontArmHighPos = 4983;
    int frontArmMedPos = frontArmHighPos;
    int frontArmTransPos = frontArmHighPos;
    final int FRONT_ARM_MAX_POS = 5500;

    int frontElbowRestPos = 0;
    int frontElbowDrivePos = 694;
    int frontElbowIntakePos = -1222;
    int frontElbowGndPos = frontElbowIntakePos;
    int frontElbowTransPos = -490; //needs a from front and from back
    int frontElbowLowPos = -289;
    int frontElbowMedPos = -1761;
    int frontElbowHighPos = -1120;
    int frontElbowAutonDrivePos = -329;


    int backArmRestPos = 0;
    int backArmDrivePos = 906;
    int backArmIntakePos = 2736;
    int backArmGndPos = backArmIntakePos;
    int backArmLowPos = backArmIntakePos;
    int backArmHighPos = 5938;
    int backArmMedPos = backArmHighPos;
    int backArmTransPos = backArmHighPos; //same
    final int BACK_ARM_MAX_POS = 6200;

    int backElbowRestPos = 0;
    int backElbowDrivePos = 673;
    int backElbowIntakePos = frontElbowIntakePos;
    int backElbowGndPos = backElbowIntakePos;
    int backElbowTransPos = -400;
    int backElbowLowPos = frontElbowLowPos;
    int backElbowMedPos = frontElbowMedPos;
    int backElbowHighPos = frontElbowHighPos;
    int backElbowAutonDrivePos = -366;

    int backHighDist = 93;
    int frontIntakeDist = 10;
    int frontTransferDist;
    int backTransferDist;

    int testingDistance = 100;
    double testingButtonValue = 0.9;

    public RobotComponents frontArmComponent;
    public RobotComponents backArmComponent;
    public RobotComponents frontElbowComponent;
    public RobotComponents backElbowComponent;

    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Declare all our motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        frontArm = hardwareMap.get(DcMotorEx.class,"frontArm");
        frontElbow = hardwareMap.get(DcMotorEx.class, "frontElbow");
        backArm = hardwareMap.get(DcMotorEx.class, "backArm");
        backElbow = hardwareMap.get(DcMotorEx.class, "backElbow");

        //declare all arm components with PID values, 435rpm motors have 384.5 ppr, 60rpm has 2786.2 ppr multiplied by gear ratio
        frontElbowComponent = new RobotComponents (frontElbow, 2786.2, 0.005, 0.2, 0.0008, 0.07);
        backElbowComponent = new RobotComponents (backElbow, 2786.2, 0.01, 0.21, 0.0007, 0.045);
        frontArmComponent = new RobotComponents (frontArm, 384.5 * 24, 0.01, 0.0, 0.0000, 0);
        backArmComponent = new RobotComponents (backArm, 384.5 * 28, 0.01, .5, 0.0003, 0);

        // Declare servos
        frontIntakeL = hardwareMap.get(CRServo.class, "intakeL");
        frontIntakeR = hardwareMap.get(CRServo.class, "intakeR");
        backIntakeL = hardwareMap.get(CRServo.class, "BackIntakeL");
        backIntakeR = hardwareMap.get(CRServo.class, "BackIntakeR");

        //declare sensors
        frontIntakeSensor = hardwareMap.get(ColorSensor.class, "CS_F");
        backIntakeSensor = hardwareMap.get(ColorSensor.class, "CS_B");
        transferSensor = hardwareMap.get(ColorSensor.class, "CS_T");
        frontButton = hardwareMap.get(TouchSensor.class, "F_Button");
        backButton = hardwareMap.get(TouchSensor.class, "B_Button");
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "F_Dist");
        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "B_Dist");

        // Camera
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // Set the direction of all our motors
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        frontArm.setDirection(DcMotorEx.Direction.FORWARD);
        frontElbow.setDirection(DcMotorEx.Direction.REVERSE);
        backElbow.setDirection(DcMotorEx.Direction.FORWARD);
        backArm.setDirection(DcMotorEx.Direction.REVERSE);

        //Sets the wheels to break on zero power
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set CRServo Directions
        frontIntakeL.setDirection(CRServo.Direction.FORWARD);
        frontIntakeR.setDirection(CRServo.Direction.REVERSE);
        backIntakeL.setDirection(CRServo.Direction.FORWARD);
        backIntakeR.setDirection(CRServo.Direction.REVERSE);

        // Run motors using encoder, so that we can move accurately. If motor doesn't have, run without encoder
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //changed to without
        frontElbow.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backElbow.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Resets encoder position to zero
        frontArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontElbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backElbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        drive = new SampleMecanumDrive(hardwareMap);
    }

    //all this does is check how far off it is from a target then returns an int to adjust encoder target
    public int moveByDistance (DistanceSensor sensor, int target) {
        if ((int) sensor.getDistance(DistanceUnit.CM) - target < 0) {
            return 5;
        } else if ((int) sensor.getDistance(DistanceUnit.CM) - target > 0) {
            return -5;
        }
        return 0;
    }

    public int moveBySetDistance (double dist, int target) {
        if ((int) dist - target < 0) {
            return 5;
        } else if ((int) dist - target > 0) {
            return -5;
        }
        return 0;
    }

    public boolean closeEnough (int pos, int target, int range) {
        if ((target - range <= pos) && (target + range >= pos)) return true;
        return false;
    }

    public boolean turnElbowOnGoingDown(RobotComponents armComponent) {
        if (armComponent.getArmAngle() < 20) {
            return true;
        } return false;
    }
    public boolean turnArmOnGoingDown(RobotComponents armComponent, RobotComponents elbowComponent) {
        if (armComponent.getArmAngle() < -20
                && !elbowComponent.motorCloseEnough(100)) { //100 is random large range
            return false;
        }
        return true;
    }
    public boolean turnElbowOnGoingUp(RobotComponents armComponent) {
        if (armComponent.getArmAngle() > 70) {
            return true;
        } return false;
    }

    public void manualDrive(double leftStickY, double leftStickX, double rightStickX) {
        //robot too fast and Jack bad at driving
        double drive = -leftStickY * .8;
        double strafe = -leftStickX * .8;
        double turn = rightStickX * .6;

        double leftFPwr;
        double rightFPwr;
        double leftBPwr;
        double rightBPwr;

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
        leftFront.setPower(leftFPwr);
        leftRear.setPower(leftBPwr);
        rightFront.setPower(rightFPwr);
        rightRear.setPower(rightBPwr);
    }

    // Function to run intake set of servos to intake a cone/transfer to other arm
    public void runIntakeServo(char servo, double power) {
        if (servo == 'F') {
            frontIntakeL.setPower(power);
            frontIntakeR.setPower(power);
        } else if (servo == 'R'){backIntakeL.setPower(power);
            backIntakeR.setPower(power);
        } else {
            frontIntakeL.setPower(power);
            frontIntakeR.setPower(power);
            backIntakeL.setPower(-power);
            backIntakeR.setPower(-power);
        }
    }
    public String returnColor(ColorSensor CS) {
        int red = CS.red();
        int green = CS.green();
        int blue = CS.blue();
        String color;

        if (red > green && red > blue && blue < 150 && green < 150) {
            color = "red";
        } else if (blue > green && red < blue && red < 150 && green < 150) {
            color = "blue";
      //  } else if (red < green && green > blue && blue < 150 && red < 150) {
        //    color = "green";
        } else {
            color = "unknown";}

        return color;
    }

    public void betterSleep(int milliseconds){
        time.reset();
        while (time.milliseconds() > milliseconds){}
        telemetry.addData("slept for ", milliseconds);
    }

    public int fixEncoderPos(DistanceSensor distance,TouchSensor button, int prevPos){
        if (distance.getDistance(DistanceUnit.CM) > testingDistance + 2 && button.getValue() != testingButtonValue){
return -10;
        }
        if (distance.getDistance(DistanceUnit.CM) < testingDistance - 2 && button.getValue() != testingButtonValue){
            return 10;
        }
        return 0;
    }

    // Function used to move any motor to different positions and hold it.
    public void move_to_position_and_hold(DcMotorEx motor, double power, int position){
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
/*        while (motor.isBusy()){
            telemetry.addData(motor +" Moving", "TRUE"); */
    }

    // drive method is used to drive using encoder positions. This is currently deprecated
    // since it is last year's code and values. If RR usage goes ary I will use it however.
    public void drive(double distanceInCm, double wheelRPower, double wheelLPower) {
        int wheelCounts = 0;
        double wCounts = distanceInCm * HWC.ONE_CM_IN_PPR;

        leftFront.setMode(STOP_AND_RESET_ENCODER);
        rightFront.setMode(STOP_AND_RESET_ENCODER);
        leftRear.setMode(STOP_AND_RESET_ENCODER);
        rightRear.setMode(STOP_AND_RESET_ENCODER);

        leftFront.setMode(RUN_WITHOUT_ENCODER);
        rightFront.setMode(RUN_WITHOUT_ENCODER);
        leftRear.setMode(RUN_WITHOUT_ENCODER);
        rightRear.setMode(RUN_WITHOUT_ENCODER);

        time.reset();

        while(Math.abs(wheelCounts) < wCounts) {
            wheelCounts = leftFront.getCurrentPosition();

            if(Math.abs(wheelCounts) < wCounts){
                leftFront.setPower(wheelLPower);
                leftRear.setPower(wheelLPower);
                rightFront.setPower(wheelRPower);
                rightRear.setPower(wheelRPower);

            }
            else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }


    public void turn(double directionInDegrees, double wheelVelocity) {
//      384.5(PPR) = ~50cm = ~20in
//      7.9(PPR) = 1cm
//      4.27(PPR) = 1 Degree
        double pprTurn = directionInDegrees * HWC.ONE_DEGREE_IN_PPR;

        if(directionInDegrees != 0) {
            if (directionInDegrees < 0) {
                leftFront.setTargetPosition(-(int) pprTurn + leftFront.getCurrentPosition());
                rightFront.setTargetPosition((int) pprTurn + rightFront.getCurrentPosition());
                leftRear.setTargetPosition(-(int) pprTurn + leftRear.getCurrentPosition());
                rightRear.setTargetPosition((int) pprTurn + rightRear.getCurrentPosition());

            } else if (directionInDegrees > 0) {
                leftFront.setTargetPosition((int) pprTurn + leftFront.getCurrentPosition());
                rightFront.setTargetPosition(-(int) pprTurn + rightFront.getCurrentPosition());
                leftRear.setTargetPosition((int) pprTurn + leftRear.getCurrentPosition());
                rightRear.setTargetPosition(-(int) pprTurn + rightRear.getCurrentPosition());
            }

            leftFront.setMode(RUN_TO_POSITION);
            rightFront.setMode(RUN_TO_POSITION);
            leftRear.setMode(RUN_TO_POSITION);
            rightRear.setMode(RUN_TO_POSITION);

            leftFront.setVelocity(wheelVelocity);
            rightFront.setVelocity(wheelVelocity);
            rightRear.setVelocity(wheelVelocity);
            leftRear.setVelocity(wheelVelocity);
        }
    }
}