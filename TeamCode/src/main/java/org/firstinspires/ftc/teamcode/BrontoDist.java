package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name="Bronto Dist", group="Iterative Opmode")

public class BrontoDist extends OpMode
{
    /** Declare OpMode members. */
    HWC bronto;
    BrontoBrain brain;
















    private ElapsedTime runtime = new ElapsedTime();

    boolean manualMode = false;

     //determines when a moving state completes and will go to

    @Override
    public void init() {
        bronto = new HWC(hardwareMap, telemetry);
        brain = new BrontoBrain(bronto);
        telemetry.addData("Status", "Initializing");


        telemetry.addData("Status", "Initialized");
    }



    /** Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY. */
    @Override
    public void init_loop() {
    }

    /** Code to run ONCE when the driver hits PLAY. */
    @Override
    public void start(){
        runtime.reset();
    }

    @Override
    public void loop() {
        /* Setup a variable for each drive wheel to save power level for telemetry. */

        telemetry.addData ("front Distance", bronto.frontDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData ("back Distance", bronto.backDistanceSensor.getDistance(DistanceUnit.CM));
        //telemetry.addData("Positions", "front Arm %d, Back Arm %d, Front Elbow %d, Back Elbow %d", bronto.frontArm.getCurrentPosition(), bronto.backArm.getCurrentPosition(), bronto.frontElbow.getCurrentPosition(), bronto.backElbow.getCurrentPosition());
        // telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", bronto.leftFront, bronto.rightFront, bronto.leftRear, bronto.rightRear, bronto.frontArm.getPower(), bronto.frontElbow.getPower());
        telemetry.update();
    }

    /** Code to run ONCE after the driver hits STOP. */
    @Override
    public void stop () {
    }
}