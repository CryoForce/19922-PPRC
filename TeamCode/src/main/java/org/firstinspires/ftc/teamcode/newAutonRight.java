package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class newAutonRight extends LinearOpMode {
    // Variables
    HWC.autonStates state = HWC.autonStates.SCANNING_FOR_SIGNAL;
    HWC.armPositions armPosition = HWC.armPositions.RESTING;
    ElapsedTime timer = new ElapsedTime();

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

            int parkingZone = bronto.sleeveDetection.getPosition();
        }

        waitForStart();
        bronto.drive.setPoseEstimate(bronto.START_POS_RIGHT);
        telemetry.addData("Status", "Running");
        telemetry.update();

        bronto.drive.followTrajectory(TC.RIGHT_deliverPreloadLeft(bronto.drive, bronto.START_POS_RIGHT));
        Pose2d newPos = TC.RIGHT_deliverPreloadLeft(bronto.drive, bronto.START_POS_RIGHT).end();
        bronto.drive.followTrajectory(TC.RIGHT_deliverPreloadForward(bronto.drive, newPos));
        newPos = TC.RIGHT_deliverPreloadForward(bronto.drive, newPos).end();
    }
}
