package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class autonArmTesting extends LinearOpMode {
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
            telemetry.addData("ZONE: ", bronto.sleeveDetection.getPosition());
            telemetry.update();

            bronto.parkingZone = bronto.sleeveDetection.getPosition();
        }

        // Set arm targets
        int backArmTarget = bronto.backArmDrivePos;
        int backElbowTarget = bronto.backElbowAutonDrivePos;
        int frontArmTarget = bronto.frontArmDrivePos;
        int frontElbowTarget = bronto.frontElbowAutonDrivePos;

        waitForStart();

        // ----------------------------- DRIVING POS ----------------------------- //
//        while(!bronto.backElbowComponent.motorCloseEnough(backElbowTarget, 20)
//                && !bronto.frontElbowComponent.motorCloseEnough(frontElbowTarget, 20)) {
//            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
//            bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
//        }
//
        // ----------------------------- DELIVERY ----------------------------- //
        // Flop out elbows
        bronto.backElbow.setPower(-0.5);
        bronto.frontElbow.setPower(0.5);
        sleep(1000);
        bronto.backElbow.setPower(0);        // Update Telemetry
        telemetry.addData("Status", "Running");
        telemetry.update();
        bronto.frontElbow.setPower(0);

        // Reset Elbow Encoders
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(!gamepad1.a) {
            sleep(1);
        }

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

        while(!gamepad1.a) {
            sleep(1);
        }

        // Move elbows (Two Stage)
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

        // Run Intake to Deliver Cone while holding elbows
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

//        // Set targets
//        backArmTarget = bronto.backArmRestPos;
//        backElbowTarget = bronto.backElbowRestPos;
//        frontArmTarget = bronto.frontArmRestPos;
//        frontElbowTarget = bronto.frontArmRestPos;
//
//        // Move using PID
//        while(!bronto.backArmComponent.motorCloseEnough(backArmTarget, 20)
//                && !bronto.backElbowComponent.motorCloseEnough(backElbowTarget, 20)
//                && !bronto.frontArmComponent.motorCloseEnough(frontArmTarget, 20)
//                && !bronto.frontElbowComponent.motorCloseEnough(frontElbowTarget, 20)) {
//            bronto.backArmComponent.moveUsingPID(backArmTarget);
//            bronto.backElbowComponent.moveUsingPID(backElbowTarget);
//            bronto.frontArmComponent.moveUsingPID(frontArmTarget);
//            bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
//        }
//
//        // Stop all motors
//        bronto.frontArm.setPower(0);
//        bronto.frontElbow.setPower(0);
//        bronto.backArm.setPower(0);
//        bronto.backElbow.setPower(0);
    }
}
