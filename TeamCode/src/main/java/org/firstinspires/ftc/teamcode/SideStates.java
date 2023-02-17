package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class SideStates {
    private DcMotorEx armMotor, elbowMotor;
    private RobotComponents armComponent, elbowComponent;
    private TouchSensor button;
    private CRServo intakeL, intakeR;
    private DistanceSensor distanceSensor;
    private ColorSensor intakeSensor;
    private HWC bronto;

    private States state;

    private boolean stateComplete = false;
    private int armRestPos, armDrivePos, armIntakePos, armGndPos, armLowPos, armMedPos, armHighPos,
            armTransPos, armMaxPos, elbowRestPos, elbowDrivePos, elbowAutonDrivePos, elbowIntakePos,
            elbowGndPos, elbowLowPos, elbowMedPos, elbowHighPos, elbowTransPos;
    private int armCloseRange = 50;
    private int elbowCloseRange = 50;

    private enum States {
        MTR,
        MTD,
        MTI,
        MTT,
        MTH,
        MTM,
        MTL,
        MTG,
        Rest,
        Drive,
        Intake,
        Transfer,
        Delivery,
        Pause,
        Unpause,
        Unknown
    }

    public SideStates (HWC initBronto, 
                       DcMotorEx initArmMotor, 
                       DcMotorEx initElbowMotor, 
                       RobotComponents initArmComponent,
                       RobotComponents initElbowComponent, 
                       TouchSensor initButton, 
                       CRServo initIntakeL,
                       CRServo initIntakeR, 
                       DistanceSensor initDistanceSensor, 
                       ColorSensor initIntakeSensor,
                       int initArmRestPos, 
                       int initArmDrivePos, 
                       int initArmIntakePos,
                       int initArmGndPos, 
                       int initArmLowPos, 
                       int initArmMedPos, 
                       int initArmHighPos,
                       int initArmTransPos, 
                       int initArmMaxPos, 
                       int initElbowRestPos, 
                       int initElbowDrivePos,
                       int initElbowAutonDrivePos, 
                       int initElbowIntakePos,
                       int initElbowGndPos,
                       int initElbowLowPos, 
                       int initElbowMedPos, 
                       int initElbowHighPos, 
                       int initElbowTransPos) {
        armComponent = initArmComponent;
        elbowComponent = initElbowComponent;
        button = initButton;
        intakeL = initIntakeL;
        intakeR = initIntakeR;
        distanceSensor = initDistanceSensor;
        bronto = initBronto;

        armRestPos = initArmRestPos;
        armDrivePos = initArmDrivePos;
        armIntakePos = initArmIntakePos;
        armGndPos = initArmGndPos;
        armLowPos = initArmMedPos;
        armHighPos = initArmHighPos;
        armTransPos = initArmTransPos;
        elbowRestPos = initElbowRestPos;
        elbowDrivePos = initElbowDrivePos;
        elbowAutonDrivePos = initElbowAutonDrivePos;
        elbowIntakePos = initElbowIntakePos;
        elbowGndPos = initElbowGndPos;
        elbowLowPos = initElbowLowPos;
        elbowMedPos = initElbowMedPos;
        elbowHighPos = initElbowHighPos;
        elbowTransPos = initElbowTransPos;
    }

    public void setState(States newState) {
        state = newState;
        //turn on elbow, only turned off in certain states
        boolean armOn = false;
        boolean elbowOn = true;

        //intake pwr vars
        double intakeLPwr = 0;
        double intakeRPwr = 0;
        
        switch (state) {
            case MTI:
                armComponent.setTarget(armIntakePos);
                elbowComponent.setTarget(elbowIntakePos);
                elbowOn = bronto.turnElbowOnGoingDown(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)) {
                    armOn = false;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                } else {
                    armOn = bronto.turnArmOnGoingDown(armComponent, elbowComponent);
                }
                break;
            case MTR:
                armComponent.setTarget(armRestPos);
                elbowComponent.setTarget(elbowRestPos);
                elbowOn = bronto.turnElbowOnGoingDown(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)
                        && button.isPressed()) {
                    armOn = false;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                } else {
                    armOn = bronto.turnArmOnGoingDown(armComponent,
                            elbowComponent);
                }
                break;

            case MTD:
                armComponent.setTarget(armDrivePos);
                elbowComponent.setTarget(elbowDrivePos);
                elbowOn = bronto.turnElbowOnGoingDown(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)) {
                    armOn = false;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                } else {
                    armOn = bronto.turnArmOnGoingDown(armComponent,
                            elbowComponent);
                }
                break;

            case MTG:
                armComponent.setTarget(armGndPos);
                elbowComponent.setTarget(elbowGndPos);
                elbowOn = bronto.turnElbowOnGoingDown(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)
                        && button.isPressed()) {
                    armOn = false;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                } else {
                    armOn = bronto.turnArmOnGoingDown(armComponent,
                            elbowComponent);
                }
                break;

            case MTH:
                armComponent.setTarget(armHighPos);
                elbowComponent.setTarget(elbowHighPos);
                elbowOn = bronto.turnElbowOnGoingUp(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)) {
                    if (button.isPressed()) {
                        armOn = false;
                        if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                            stateComplete = true;
                        }
                    } else {
                        armComponent.incrementTarget((armMaxPos/Math.abs(armMaxPos))*20);
                    }
                } else {armOn = true;}
                break;

            case MTL:

                break;

            case MTM:
                break;

            case MTT:
                armComponent.setTarget(armTransPos);
                elbowComponent.setTarget(elbowTransPos);
                elbowOn = bronto.turnElbowOnGoingUp(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)) {
                    if (button.isPressed()) {
                        armOn = false;
                        if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                            stateComplete = true;
                        }
                    } else {
                        armComponent.incrementTarget((armMaxPos/Math.abs(armMaxPos))*20);
                    }
                } else {armOn = true;}
                break;

            case Rest:
                break;

            case Drive:
                break;

            case Transfer:
                intakeLPwr = -1;
                intakeRPwr = -1;
                if (bronto.returnColor(intakeSensor) != "unknown") {
                    stateComplete = true;
                }
                break;

            case Delivery:
                intakeLPwr = -1;
                intakeRPwr = -1;
                if (bronto.returnColor(intakeSensor) != "unknown") {
                    intakeLPwr = 0;
                    intakeRPwr = 0;
                    stateComplete = true;
                }
                break;

            case Unknown:
                break;

            default:
                state = States.Unknown;
                break;

        }

        if (Math.abs(armComponent.getTarget()) < Math.abs(armMaxPos)) {
            armComponent.setTarget(armMaxPos);
        } //TODO: figure out way to make this also work for 0 limit

        //after going through every state to determine what is close for the arms, set to 0 if they are close enough
        if (!armOn) {armMotor.setPower(0);}
        else {armComponent.moveUsingPID();}
        if (!elbowOn) {elbowMotor.setPower(0);}
        else {elbowComponent.moveUsingPID();}
    }

    public boolean getCompletionStatus () {
        return stateComplete;
    }

    public String returnColor() {
        return bronto.returnColor(intakeSensor);
    }
}
