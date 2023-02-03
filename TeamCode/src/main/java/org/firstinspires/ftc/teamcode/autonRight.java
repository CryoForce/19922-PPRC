package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class autonRight extends LinearOpMode {

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
        telemetry.addData("Arm Position", "Init");
        telemetry.update();

        // Get Computer Vision from Signal Cone
        brain.cv();
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", bronto.sleeveDetection.getPosition());
            telemetry.update();

            bronto.parkingZone = bronto.sleeveDetection.getPosition();
        }

        waitForStart();
        // Set Starting Position Estimate for RoadRunner
        bronto.drive.setPoseEstimate(bronto.START_POS_RIGHT);
        // Update Telemetry
        telemetry.addData("Status", "Running");
        telemetry.update();

        // Drive to location to deliver preload cone
        bronto.drive.followTrajectory(TC.RIGHT_deliverPreloadLeft(bronto.drive, bronto.START_POS_RIGHT));
        Pose2d newPos = TC.RIGHT_deliverPreloadLeft(bronto.drive, bronto.START_POS_RIGHT).end();
        bronto.drive.followTrajectory(TC.RIGHT_deliverPreloadForward(bronto.drive, newPos));
        newPos = TC.RIGHT_deliverPreloadForward(bronto.drive, newPos).end();

        // Flop out elbows
        // TODO: Check power vals for this:
        bronto.backElbow.setPower(0.5);
        bronto.frontElbow.setPower(0.5);
        sleep(1000);
        bronto.backElbow.setPower(0);
        bronto.frontElbow.setPower(0);

        // Reset Elbow Encoders
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set arm targets
        int backArmTarget = bronto.backArmHighPos;
        int backElbowTarget = bronto.backElbowHighPos;
        int frontArmTarget = 0;
        int frontElbowTarget = 0;

        // Move arm using PID
        while(!bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)) {
            bronto.backArmComponent.moveUsingPID(backArmTarget);
        }

        while(!bronto.backButton.isPressed()) {
            backArmTarget -= 20;
            bronto.backArmComponent.moveUsingPID(backArmTarget);
        }

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

        // Stop Intake
        bronto.backIntakeL.setPower(0);
        bronto.backIntakeR.setPower(0);

        // TODO: Move arms to drive pos

        // Drive to parking location based off of CV from earlier
        bronto.drive.followTrajectory(TC.RIGHT_forwardToPark(bronto.drive, newPos));
        newPos = TC.RIGHT_forwardToPark(bronto.drive, newPos).end();
        if(bronto.parkingZone == 1) {
            bronto.drive.followTrajectory(TC.RIGHT_parkingZone1(bronto.drive, newPos));
            newPos = TC.RIGHT_parkingZone1(bronto.drive, newPos).end();
        } else if(bronto.parkingZone == 2) {
            bronto.drive.followTrajectory(TC.RIGHT_parkingZone2(bronto.drive, newPos));
            newPos = TC.RIGHT_parkingZone2(bronto.drive, newPos).end();
        } else if(bronto.parkingZone == 3) {
            bronto.drive.followTrajectory(TC.RIGHT_parkingZone3(bronto.drive, newPos));
            newPos = TC.RIGHT_parkingZone3(bronto.drive, newPos).end();
        } else {
            bronto.drive.followTrajectory(TC.RIGHT_parkingZone1(bronto.drive, newPos));
            newPos = TC.RIGHT_parkingZone1(bronto.drive, newPos).end();
        }

        // Set Arm Targets to restPos
        backArmTarget = bronto.backArmRestPos;
        backElbowTarget = bronto.backElbowRestPos;
        frontArmTarget = bronto.frontArmRestPos;
        frontElbowTarget = bronto.frontElbowRestPos;

        // Move arms using PID
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
