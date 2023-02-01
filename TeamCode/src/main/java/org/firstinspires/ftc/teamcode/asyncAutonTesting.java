package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class asyncAutonTesting extends LinearOpMode {
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
        bronto.drive.setPoseEstimate(bronto.START_POS_LEFT);

        // Update Telemetry
        telemetry.addData("Status", "Running");
        telemetry.update();

        // ------------------- FORWARD TO OPEN ARMS ------------------- //
        bronto.drive.followTrajectoryAsync(TC.forwardToOpenArms(bronto.drive, bronto.START_POS_LEFT));
        Pose2d newPos = TC.forwardToOpenArms(bronto.drive, bronto.START_POS_LEFT).end();

        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
        }

        // ------------------- STRAFE RIGHT ------------------- //
        bronto.drive.followTrajectoryAsync(TC.LEFT_deliverPreloadRight(bronto.drive, newPos));
        newPos = TC.LEFT_deliverPreloadRight(bronto.drive, newPos).end();

        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
        }

        // ------------------- DRIVE FORWARD ------------------- //
        bronto.drive.followTrajectoryAsync(TC.LEFT_deliverPreloadForward(bronto.drive, newPos));
        newPos = TC.LEFT_deliverPreloadForward(bronto.drive, newPos).end();

        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
        }

        // ------------------- FORWARD TO PARK ------------------- //
        bronto.drive.followTrajectoryAsync(TC.LEFT_forwardToPark(bronto.drive, newPos));
        newPos = TC.LEFT_forwardToPark(bronto.drive, newPos).end();

        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
        }

        // ------------------- STRAFE TO PARKING ZONE ------------------- //
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

        bronto.drive.update();
        while(bronto.drive.isBusy()) {
            bronto.drive.update();
        }
    }
}