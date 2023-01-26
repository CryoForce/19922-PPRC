package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class newAutonLeft extends LinearOpMode {
        // Variables

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

            brain.cv();
            while (!isStarted()) {
                telemetry.addData("ROTATION: ", bronto.sleeveDetection.getPosition());
                telemetry.update();

                bronto.parkingZone = bronto.sleeveDetection.getPosition();
            }

            waitForStart();
            bronto.drive.setPoseEstimate(bronto.START_POS_LEFT);
            telemetry.addData("Status", "Running");
            telemetry.update();

            bronto.drive.followTrajectory(TC.LEFT_deliverPreloadRight(bronto.drive, bronto.START_POS_LEFT));
            Pose2d newPos = TC.LEFT_deliverPreloadRight(bronto.drive, bronto.START_POS_LEFT).end();
            bronto.drive.followTrajectory(TC.LEFT_deliverPreloadForward(bronto.drive, newPos));
            newPos = TC.LEFT_deliverPreloadForward(bronto.drive, newPos).end();

            // TODO: ADD ARM MOVEMENTS

            bronto.drive.followTrajectory(TC.LEFT_forwardToPark(bronto.drive, newPos));
            newPos = TC.LEFT_forwardToPark(bronto.drive, newPos).end();
            if(bronto.parkingZone == 1) {
                bronto.drive.followTrajectory(TC.LEFT_parkingZone1(bronto.drive, newPos));
                newPos = TC.LEFT_parkingZone1(bronto.drive, newPos).end();
            } else if(bronto.parkingZone == 2) {
                bronto.drive.followTrajectory(TC.LEFT_parkingZone2(bronto.drive, newPos));
                newPos = TC.LEFT_parkingZone2(bronto.drive, newPos).end();
            } else if(bronto.parkingZone == 3) {
                bronto.drive.followTrajectory(TC.LEFT_parkingZone3(bronto.drive, newPos));
                newPos = TC.LEFT_parkingZone3(bronto.drive, newPos).end();
            } else {
                bronto.drive.followTrajectory(TC.LEFT_parkingZone1(bronto.drive, newPos));
                newPos = TC.LEFT_parkingZone1(bronto.drive, newPos).end();
            }
        }
    }
