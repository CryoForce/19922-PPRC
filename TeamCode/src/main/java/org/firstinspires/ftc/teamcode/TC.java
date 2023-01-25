package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TC {
    public static Trajectory RIGHT_deliverPreloadLeft(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeLeft(36)
                .build();
    }
    public static Trajectory RIGHT_deliverPreloadForward(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .forward(13)
                .build();
    }
    //parking zone trajectory code 1-3
    public static Trajectory RIGHT_forwardToPark(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .forward(11)
                .build();
    }
    public static Trajectory RIGHT_parkingZone1(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeRight(11)
                .build();
    }
    public static Trajectory RIGHT_parkingZone2(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeRight(36)
                .build();
    }public static Trajectory RIGHT_parkingZone3(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeRight(59)
                .build();
    }


    // LEFT movment stuff for auton
    public static Trajectory LEFT_deliverPreloadRight(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeRight(36)
                .build();
    }
    public static Trajectory LEFT_deliverPreloadForward(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .forward(13)
                .build();
    }
    //parking zone trajectory code 1-3
    public static Trajectory LEFT_forwardToPark(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .forward(11)
                .build();
    }
    public static Trajectory LEFT_parkingZone1(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeLeft(11)
                .build();
    }
    public static Trajectory LEFT_parkingZone2(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeLeft(36)
                .build();
    }public static Trajectory LEFT_parkingZone3(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeLeft(59)
                .build();
    }
}