package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotComponents {

    private final DcMotorEx motor;
    private final double ticks_in_degrees;
    public final double ticks_per_degree;
    private final double F;
    private final PIDController controller;
    public final double armLength = 38.4; //cm, random atm TODO: check arm lengths
    public final double elbowLength = 19.2; //cm

    RobotComponents (DcMotorEx motor, double ticks_in_degrees, double p, double i, double d, double f) {
        this.motor = motor;
        this.ticks_in_degrees = ticks_in_degrees;
        this.F = f;
        ticks_per_degree = ticks_in_degrees/360.0;

        controller = new PIDController (p,i,d);
    }

    public void moveUsingPID (int target) {

        controller.reset();
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        int armPos = motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * F;

        double power = pid + ff;

        motor.setPower(power);

    }

    public boolean motorCloseEnough(int target, int range) {
        if ((target - range <= motor.getCurrentPosition()) && (target + range >= motor.getCurrentPosition())) return true;
        return false;
    }

    public double armTicksUsingCoords (double x, double y) {
        double dist = Math.sqrt(x*x + y*y);
        double d1 = Math.toDegrees(Math.atan2(y, x));
        double d2 = Math.toDegrees(lawOfCosines(dist, armLength, elbowLength));

        double a1 = d1+d2;
        /*
        not confident about this but should give the encoder position correctly based on angle
        divides by 4 because that gives the 0 degrees (90 degrees relative to the arm), multiplied by said value and added to it
         */
        return (ticks_per_degree * a1 + (ticks_in_degrees / 4.0));
    }

    public double armTicksUsingAngle (double a) {
        return (ticks_per_degree * a + (ticks_in_degrees / 4.0));
    }

    public double armAngleUsingCoords (double x, double y) {
        double dist = Math.sqrt(x*x + y*y);
        double d1 = Math.toDegrees(Math.atan2(y, x));
        double d2 = Math.toDegrees(lawOfCosines(dist, armLength, elbowLength));

        double a1 = d1+d2;

        return a1;
    }

    public double elbowTicksUsingCoords (double x, double y) {
        double dist = Math.sqrt(x*x + y*y);
        double a2 = Math.toDegrees(lawOfCosines(armLength, elbowLength, dist));
        return (ticks_per_degree * a2); //this definitely needs more math to account for arm movement, but I'm unsure on how
    }

    private double lawOfCosines (double a, double b, double c) { //return angle given side lengths using law of cosines
        return Math.acos((a*a + b*b - c*c) / (2 * a * b));
    }
}
