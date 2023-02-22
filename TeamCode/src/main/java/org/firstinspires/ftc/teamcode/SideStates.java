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

    //stateComplete represents when a state is complete for terry
    private boolean stateComplete = false;
    //stateReady represents whether or not the state is ready to start the other side movement
    //not necesarilly done (for example arm stopped moving so second one can start)
    private boolean stateReady = false;
    //boolean to represent master approval for continuation
    private boolean observerApproval = false;
    private int armRestPos, armDrivePos, armIntakePos, armGndPos, armLowPos, armMedPos, armHighPos,
            armTransPos, armMaxPos, elbowRestPos, elbowDrivePos, elbowAutonDrivePos, elbowIntakePos,
            elbowGndPos, elbowLowPos, elbowMedPos, elbowHighPos, elbowTransPos;
    private int armCloseRange = 50;
    private int elbowCloseRange = 50;

    public enum States {
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
        armMotor = initArmMotor;
        elbowMotor = initElbowMotor;
        armComponent = initArmComponent;
        elbowComponent = initElbowComponent;
        button = initButton;
        intakeL = initIntakeL;
        intakeR = initIntakeR;
        distanceSensor = initDistanceSensor;
        intakeSensor = initIntakeSensor;
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

    public void setArmState(States newState) {
        state = newState;
        //turn on elbow, only turned off in certain states
        boolean armOn = false;
        boolean elbowOn = true;

        //complete and ready reset
        stateReady = false;
        stateComplete = false;
        observerApproval = false;

        //intake pwr vars
        double intakePwr = 0;
        
        switch (state) {
            case MTI:
                armOn = true;
                armComponent.setTarget(armIntakePos);
                elbowComponent.setTarget(elbowIntakePos);
                elbowOn = bronto.turnElbowOnGoingDown(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)) {
                    armOn = false;
                    stateReady = true;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                } else {
                    armOn = bronto.turnArmOnGoingDown(armComponent, elbowComponent);
                }
                break;
            case MTR:
                armOn = true;
                armComponent.setTarget(armRestPos);
                elbowComponent.setTarget(elbowRestPos);
                elbowOn = bronto.turnElbowOnGoingDown(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)
                        && button.isPressed()) {
                    armOn = false;
                    stateReady = true;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                } else {
                    armOn = bronto.turnArmOnGoingDown(armComponent,
                            elbowComponent);
                }
                break;

            case MTD:
                armOn = true;
                armComponent.setTarget(armDrivePos);
                elbowComponent.setTarget(elbowDrivePos);
                elbowOn = bronto.turnElbowOnGoingDown(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)) {
                    armOn = false;
                    stateReady = true;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                } else {
                    armOn = bronto.turnArmOnGoingDown(armComponent,
                            elbowComponent);
                }
                break;

            case MTG:
                armOn = true;
                armComponent.setTarget(armGndPos);
                elbowComponent.setTarget(elbowGndPos);
                elbowOn = bronto.turnElbowOnGoingDown(armComponent);
                if (armComponent.motorCloseEnough(armCloseRange)
                        && button.isPressed()) {
                    armOn = false;
                    stateReady = true;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                } else {
                    armOn = bronto.turnArmOnGoingDown(armComponent,
                            elbowComponent);
                }
                break;

            case MTH:
                armOn = true;
                armComponent.setTarget(5683);
                elbowComponent.setTarget(elbowHighPos);
                stateReady = bronto.turnElbowOnGoingUp(armComponent);
                elbowOn = stateReady; //observer approval not needed atm
                        //&& observerApproval;

                if (armComponent.motorCloseEnough(armCloseRange)) {
                    if (button.isPressed()) {
                        armOn = false;
                        if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                            stateComplete = true;
                        }
                    } else {
                        armComponent.incrementTarget(20);
                    }
                } else {armOn = true;}
                break;

            case MTL:
                armOn = true;
                armComponent.setTarget(armLowPos);
                elbowComponent.setTarget(elbowLowPos);
                stateReady = bronto.turnElbowOnGoingUp(armComponent);
                elbowOn = stateReady;
                if (armComponent.motorCloseEnough(armCloseRange)) {
                    armOn = false;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                }
                break;

            case MTM:
                armOn = true;
                armComponent.setTarget(armMedPos);
                elbowComponent.setTarget(elbowMedPos);
                stateReady = bronto.turnElbowOnGoingUp(armComponent);
                elbowOn = stateReady;
                if (armComponent.motorCloseEnough(armCloseRange)) {
                    armOn = false;
                    if (elbowComponent.motorCloseEnough(elbowCloseRange)) {
                        stateComplete = true;
                    }
                }
                break;

            case MTT:
                armOn = true;
                armComponent.setTarget(armTransPos);
                elbowComponent.setTarget(elbowTransPos);
                stateReady = bronto.turnElbowOnGoingDown(armComponent);
                elbowOn = stateReady; //observer approval not needed atm
                        //&& observerApproval;
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
                intakePwr = -1;
                if (bronto.returnColor(intakeSensor) != "unknown") {
                    stateComplete = true;
                    if (observerApproval) {
                        intakePwr = 0;
                    }
                }
                break;

            case Delivery:
                intakePwr = -1;
                if (bronto.returnColor(intakeSensor) != "unknown") {
                    intakePwr = 0;
                    stateComplete = true;
                }
                break;

            case Pause:
                break;

            case Unknown:
                break;

            default:
                state = States.Unknown;
                break;

        }

        //armComponent.setTarget(Range.clip(armComponent.getTarget(), 0, armMaxPos)); //prevents from going past max

        //set servo pwr
        intakeL.setPower(intakePwr);
        intakeR.setPower(intakePwr);

        //after going through every state to determine what is close for the arms, set to 0 if they are close enough
        if (!armOn) {armMotor.setPower(0);}
        else {armComponent.moveUsingPID();}
        if (!elbowOn) {elbowMotor.setPower(0);}
        else {elbowComponent.moveUsingPID();}
    }

    public States getCurrentState () {return state;}

    public void setObserverApproval(boolean approval) {observerApproval = approval;}

    public boolean getCompletionStatus () {return stateComplete;}
    public boolean getReadinessStatus () {return stateReady;}

    public String returnColor() {
        return bronto.returnColor(intakeSensor);
    }
}
