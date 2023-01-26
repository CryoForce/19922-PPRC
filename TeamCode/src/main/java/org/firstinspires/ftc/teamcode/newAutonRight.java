package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class newAutonRight extends LinearOpMode {

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
        bronto.drive.setPoseEstimate(bronto.START_POS_RIGHT);
        telemetry.addData("Status", "Running");
        telemetry.update();

        bronto.drive.followTrajectory(TC.RIGHT_deliverPreloadLeft(bronto.drive, bronto.START_POS_RIGHT));
        Pose2d newPos = TC.RIGHT_deliverPreloadLeft(bronto.drive, bronto.START_POS_RIGHT).end();
        bronto.drive.followTrajectory(TC.RIGHT_deliverPreloadForward(bronto.drive, newPos));
        newPos = TC.RIGHT_deliverPreloadForward(bronto.drive, newPos).end();

        // TODO: ADD ARM MOVEMENTS

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
    }
}
