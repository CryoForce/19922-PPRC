package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class stateFinalAuton extends LinearOpMode {
    public static HWC.START_POSITION startPosition;
    HWC bronto;
    public int backElbowTarget, frontElbowTarget, backArmTarget, frontArmTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        bronto = new HWC(hardwareMap, telemetry);
        BrontoBrain brain = new BrontoBrain(bronto);

        backElbowTarget = bronto.backElbowAutonDrivePos;
        frontElbowTarget = bronto.frontElbowAutonDrivePos;
        backArmTarget = bronto.backArmDrivePos;
        frontArmTarget = bronto.frontArmDrivePos;

        selectStartingPosition();

        //Build Autonomous trajectory to be used based on starting position selected
        buildAuto();
        bronto.drive.getLocalizer().setPoseEstimate(initPose);

        while(!isStopRequested() && !opModeIsActive()) {
            brain.cv();
            bronto.parkingZone = bronto.sleeveDetection.getPosition();

            telemetry.clearAll();
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Vision identified Parking Location", bronto.parkingZone);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            buildParking();

            //run Autonomous trajectory
            runAutoAndParking();
        }

        //Trajectory is completed, display Parking complete
        parkingComplete();
    }

    //Method to select starting position using X, Y, A, B buttons on controller
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Select Starting Position using X,Y,A,B Keys on Controller 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = HWC.START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = HWC.START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = HWC.START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = HWC.START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //Initialize TrajectorySequences
    TrajectorySequence trajectoryAuto, trajectoryParking ;

    //Initialize Pose2d's
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d pickConePose;
    Pose2d dropConePose0, dropConePose1, dropConePose2;
    Pose2d parkPose;

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(-12, 12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(-11, 12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(-10, 12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pose
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(-12, -12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(-11, -12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(-10, -12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                break;
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(12, -12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(11, -12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(10, -15, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                break;
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(12, 12, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(11, 12, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(10, 12, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = bronto.drive.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(midWayPose)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(
                        0.5* DriveConstants.MAX_VEL/*Slower velocity*/,
                        0.5*DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
                        DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(dropConePose0)
                .addDisplacementMarker(() -> {
                    dropCone(0); //Drop preloaded Cone
                })
                //Uncomment following line to stop reduction in speed. And move to the position after which you want to stop reducing speed.
                //.resetVelConstraint()
//                .lineToLinearHeading(pickConePose)
//                .addDisplacementMarker(() -> {
//                    pickCone(1); //Pick top cone from stack
//                })
//                .lineToLinearHeading(dropConePose1)
//                .addDisplacementMarker(() -> {
//                    dropCone(1); //Drop cone on junction
//                })
//                .lineToLinearHeading(pickConePose)
//                .addDisplacementMarker(() -> {
//                    pickCone(2); //Pick second cone from stack
//                })
//                .lineToLinearHeading(dropConePose2)
//                .addDisplacementMarker(() -> {
//                    dropCone(2); //Drop cone on junction
//                })
                .build();
    }

    //Build parking trajectory based on target detected by vision
    public void buildParking() {
        switch (startPosition) {
            case BLUE_LEFT:
                switch(bronto.parkingZone) {
                    case 1: parkPose = new Pose2d(-12, 60, Math.toRadians(180)); break; // Location 1
                    case 2: parkPose = new Pose2d(-12, 36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, 11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                switch(bronto.parkingZone) {
                    case 1: parkPose = new Pose2d(-12, -11, Math.toRadians(180)); break; // Location 1
                    case 2: parkPose = new Pose2d(-12, -36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, -60, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_LEFT:
                switch(bronto.parkingZone){
                    case 1: parkPose = new Pose2d(12, -60, Math.toRadians(0)); break; // Location 1
                    case 2: parkPose = new Pose2d(12, -36, Math.toRadians(0)); break; // Location 2
                    case 3: parkPose = new Pose2d(12, -11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch(bronto.parkingZone){
                    case 1: parkPose = new Pose2d(12, 11, Math.toRadians(0)); break; // Location 1
                    case 2: parkPose = new Pose2d(12, 36, Math.toRadians(0)); break; // Location 2
                    case 3: parkPose = new Pose2d(12, 60, Math.toRadians(0)); break; // Location 3
                }
                break;
        }

        trajectoryParking = bronto.drive.trajectorySequenceBuilder(midWayPose)
                .lineToLinearHeading(parkPose)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking(){
        telemetry.setAutoClear(false);
        telemetry.update();
        //Run the trajectory built for Auto and Parking
//        bronto.drive.followTrajectorySequence(trajectoryAuto);
//        bronto.drive.followTrajectorySequence(trajectoryParking);
        bronto.drive.followTrajectorySequenceAndArms(trajectoryAuto, bronto, backElbowTarget, frontElbowTarget, backArmTarget, frontArmTarget);
        bronto.drive.followTrajectorySequenceAndArms(trajectoryParking, bronto, backElbowTarget, frontElbowTarget, backArmTarget, frontArmTarget);
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount){
        /*TODO: Add code to drop cone on junction*/
        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }

    public void parkingComplete(){
        telemetry.addData("Parked in Location", bronto.parkingZone);
        telemetry.update();
    }
}
