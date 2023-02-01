package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class asyncAutonLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Tell driver bronto is initializing
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Run any initialization code necessary
        HWC bronto = new HWC(hardwareMap, telemetry);
        BrontoBrain brain = new BrontoBrain(bronto);

        // Tell driver bronto is ready and waiting for start
        telemetry.addData("Status", "Initialized - Waiting for Start");
        telemetry.update();

        // Get Computer Vision from Signal Cone
        brain.cv();
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", bronto.sleeveDetection.getPosition());
            telemetry.update();

            bronto.parkingZone = bronto.sleeveDetection.getPosition();
        }

        // Set arm targets
        int backArmTarget = bronto.backArmDrivePos;
        int backElbowTarget = bronto.backElbowDrivePos;
        int frontArmTarget = bronto.frontArmDrivePos;
        int frontElbowTarget = bronto.frontElbowDrivePos;

        // Wait for Drivers to start
        waitForStart();

        // Set Starting Position Estimate for RoadRunner
        bronto.drive.setPoseEstimate(bronto.START_POS_LEFT);

        // Update Telemetry
        telemetry.addData("Status", "Running");
        telemetry.update();

        // ------------------- FORWARD TO OPEN ARMS ------------------- //
        // Move Forward to open the arms
        bronto.drive.followTrajectoryAsync(TC.forwardToOpenArms(bronto.drive, bronto.START_POS_LEFT));
        Pose2d newPos = TC.forwardToOpenArms(bronto.drive, bronto.START_POS_LEFT).end();

        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
        }

        // Move arms out slightly
        while(!bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)
                && !bronto.backElbowComponent.motorCloseEnough(backElbowTarget, 20)
                && !bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20)
                && !bronto.frontElbowComponent.motorCloseEnough(frontElbowTarget, 20)) {
            bronto.backArmComponent.moveUsingPID(backArmTarget);
            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
            bronto.frontArmComponent.moveUsingPID(frontArmTarget);
            bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
        }

        // Move while holding Elbows out
        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
            bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
        }

        // ----------------------------- STRAFE LEFT ----------------------------- //
        // Set Trajectory
        bronto.drive.followTrajectoryAsync(TC.LEFT_deliverPreloadRight(bronto.drive, newPos));
        newPos = TC.LEFT_deliverPreloadRight(bronto.drive, newPos).end();

        // Move while holding Elbows out
        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
            bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
        }

        // ----------------------------- FORWARD TO DELIVER ----------------------------- //
        // Set Trajectory
        bronto.drive.followTrajectoryAsync(TC.LEFT_deliverPreloadForward(bronto.drive, newPos));
        newPos = TC.LEFT_deliverPreloadForward(bronto.drive, newPos).end();

        // Move while holding Elbows out
        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
            bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
        }

        // ----------------------------- FLOP OUT ELBOWS & RESET ENCODERS ----------------------------- //
        // TODO: Check power vals for this:
        bronto.backElbow.setPower(0.5);
        bronto.frontElbow.setPower(0.5);
        sleep(1000);
        bronto.backElbow.setPower(0);
        bronto.frontElbow.setPower(0);

        // Reset Elbow Encoders
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ----------------------------- DELIVER CONE ----------------------------- //
        // ------------ ARM MOVE ------------ //
        // Set Targets
        backArmTarget = bronto.backArmHighPos;
        backElbowTarget = bronto.backElbowHighPos;

        // Move arm using PID (Two Stage)
        while(!bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)) {
            bronto.backArmComponent.moveUsingPID(backArmTarget);
        }
        while(!bronto.backButton.isPressed()) {
            backArmTarget -= 20;
            bronto.backArmComponent.moveUsingPID(backArmTarget);
        }

        // ------------ ELBOW MOVE ------------ //
        // Move elbow using PID & Distance Sensors
        double distAvg = bronto.backHighDist;
        double [] distances = new double[3];

        while(!bronto.backElbowComponent.motorCloseEnough(backElbowTarget, 20)) {
            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
        }
        while(!bronto.backElbowComponent.motorCloseEnough(backElbowTarget, 20)
                && !bronto.closeEnough((int) distAvg, bronto.backHighDist, 1)) {
            backElbowTarget += bronto.moveBySetDistance(distAvg, bronto.backHighDist);
            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
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
        }

        // ------------ RUN INTAKE WHILE HOLDING ELBOWS ------------ //
        // Run Intake to Deliver Cone
        while(bronto.returnColor(bronto.backIntakeSensor) != "unknown") {
            bronto.backIntakeL.setPower(-1);
            bronto.backIntakeR.setPower(-1);
            backElbowTarget += bronto.moveBySetDistance(distAvg, bronto.backHighDist);
            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
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
        }

        // ------------ STOP INTAKES ------------ //
        // Stop Intake
        bronto.backIntakeL.setPower(0);
        bronto.backIntakeR.setPower(0);

        // ----------------------------- PARKING USING CV VALUE ----------------------------- //
        // Set Trajectory to follow (forward to park)
        bronto.drive.followTrajectoryAsync(TC.LEFT_forwardToPark(bronto.drive, newPos));
        newPos = TC.LEFT_forwardToPark(bronto.drive, newPos).end();

        // Hold frontElbow at driving pos and follow trajectory
        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
            bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
        }

        // Check Parking zone and set trajectory to follow
        if(bronto.parkingZone == 1) {
            bronto.drive.followTrajectoryAsync(TC.LEFT_parkingZone1(bronto.drive, newPos));
            newPos = TC.LEFT_parkingZone1(bronto.drive, newPos).end();
        } else if (bronto.parkingZone == 2) {
            bronto.drive.followTrajectory((TC.LEFT_parkingZone2(bronto.drive, newPos)));
            newPos = TC.LEFT_parkingZone2(bronto.drive, newPos).end();
        } else if (bronto.parkingZone == 3) {
            bronto.drive.followTrajectory(TC.LEFT_parkingZone3(bronto.drive, newPos));
            newPos = TC.LEFT_parkingZone3(bronto.drive, newPos).end();
        } else {
            bronto.drive.followTrajectory(TC.LEFT_parkingZone1(bronto.drive, newPos));
            newPos = TC.LEFT_parkingZone1(bronto.drive, newPos).end();
        }

        // ----------------------------- LOWER ARMS ----------------------------- //
        // Set targets
        backArmTarget = bronto.backArmRestPos;
        backElbowTarget = bronto.backElbowRestPos;
        frontArmTarget = bronto.frontArmRestPos;
        frontElbowTarget = bronto.frontArmRestPos;

        // Move using PID
        while(!bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)
                && !bronto.backElbowComponent.motorCloseEnough(backElbowTarget, 20)
                && !bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20)
                && !bronto.frontElbowComponent.motorCloseEnough(frontElbowTarget, 20)) {
            bronto.backArmComponent.moveUsingPID(backArmTarget);
            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
            bronto.frontArmComponent.moveUsingPID(frontArmTarget);
            bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
        }

        // Stop all motors
        bronto.frontArm.setPower(0);
        bronto.frontElbow.setPower(0);
        bronto.backArm.setPower(0);
        bronto.backElbow.setPower(0);
    }
}