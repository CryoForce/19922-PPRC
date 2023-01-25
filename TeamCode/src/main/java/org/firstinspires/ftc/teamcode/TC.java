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
}