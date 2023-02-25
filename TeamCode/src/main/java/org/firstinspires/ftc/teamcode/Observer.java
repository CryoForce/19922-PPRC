package org.firstinspires.ftc.teamcode;

public class Observer {
    private RobotStates robotState;
    private SideStates trueFront;
    private SideStates trueBack;
    private SideStates frontSide;
    private SideStates backSide;
    private HWC bronto;

    public enum RobotStates {
        Rest,
        Drive,
        Intake,
        Transfer,
        GndJct,
        LowJct,
        MedJct,
        HighJct,
        Unknown
    }

    public Observer (HWC initBronto) {
        this.bronto = initBronto;
        trueFront = new SideStates(bronto,
                bronto.frontArm,
                bronto.frontElbow,
                bronto.frontArmComponent,
                bronto.frontElbowComponent,
                bronto.frontButton,
                bronto.frontIntakeL,
                bronto.frontIntakeR,
                bronto.frontDistanceSensor,
                bronto.frontIntakeSensor,
                bronto.frontArmRestPos,
                bronto.frontArmDrivePos,
                bronto.frontArmIntakePos,
                bronto.frontArmGndPos,
                bronto.frontArmLowPos,
                bronto.frontArmMedPos,
                bronto.frontArmHighPos,
                bronto.frontArmTransPos,
                bronto.FRONT_ARM_MAX_POS,
                bronto.frontElbowRestPos,
                bronto.frontElbowDrivePos,
                bronto.frontElbowAutonDrivePos,
                bronto.frontElbowIntakePos,
                bronto.frontElbowGndPos,
                bronto.frontElbowLowPos,
                bronto.frontElbowMedPos,
                bronto.frontElbowHighPos,
                bronto.frontElbowTransPos);
        trueBack = new SideStates(bronto,
                bronto.backArm,
                bronto.backElbow,
                bronto.backArmComponent,
                bronto.backElbowComponent,
                bronto.backButton,
                bronto.backIntakeL,
                bronto.backIntakeR,
                bronto.backDistanceSensor,
                bronto.backIntakeSensor,
                bronto.backArmRestPos,
                bronto.backArmDrivePos,
                bronto.backArmIntakePos,
                bronto.backArmGndPos,
                bronto.backArmLowPos,
                bronto.backArmMedPos,
                bronto.backArmHighPos,
                bronto.backArmTransPos,
                bronto.BACK_ARM_MAX_POS,
                bronto.backElbowRestPos,
                bronto.backElbowDrivePos,
                bronto.backElbowAutonDrivePos,
                bronto.backElbowIntakePos,
                bronto.backElbowGndPos,
                bronto.backElbowLowPos,
                bronto.backElbowMedPos,
                bronto.backElbowHighPos,
                bronto.backElbowTransPos);
        frontSide = trueFront;
        backSide = trueBack;
    }
    public void setCycleState (RobotStates newState) {
        robotState = newState;
        switch (robotState) {
            case Rest:
                frontSide.setArmState(SideStates.States.MTR);
                backSide.setArmState(SideStates.States.MTR);
                if (frontSide.getCompletionStatus() && backSide.getCompletionStatus()) {
                    frontSide.setArmState(SideStates.States.Rest);
                    backSide.setArmState(SideStates.States.Rest);
                }
                break;
            case Drive:
                frontSide.setArmState(SideStates.States.MTD);
                backSide.setArmState(SideStates.States.MTD);
                if (frontSide.getCompletionStatus() && backSide.getCompletionStatus()) {
                    robotState = RobotStates.Unknown;
                }
                break;
            case Intake:
                /*
                if MTI, set again and if ready set back MTT, if done set front Intake
                if Intake, set again and set back MTT, if complete move to unknown state
                else set MTI (for the first time)
                 */
                if (frontSide.getCurrentState() == SideStates.States.MTI) {
                    frontSide.setArmState(SideStates.States.MTI);
                    if (frontSide.getReadinessStatus()) {
                        backSide.setArmState(SideStates.States.MTT);
                    }
                    if (frontSide.getCompletionStatus()) {
                        frontSide.setArmState(SideStates.States.Intake);
                    }
                } else if (frontSide.getCurrentState() == SideStates.States.Intake){
                    frontSide.setArmState(SideStates.States.Intake);
                    backSide.setArmState(SideStates.States.MTT);
                    if (frontSide.getCompletionStatus()) {
                        robotState = RobotStates.Unknown;
                    }
                } else frontSide.setArmState(SideStates.States.MTI);
                break;
            case Transfer:
                if (frontSide.getCurrentState() == SideStates.States.MTT) {
                    frontSide.setArmState(SideStates.States.MTT);
                    if (frontSide.getReadinessStatus()) {
                        backSide.setArmState(SideStates.States.MTT);
                    }
                    if (frontSide.getCompletionStatus() && backSide.getCompletionStatus()) {
                        frontSide.setArmState(SideStates.States.Transfer);
                        backSide.setArmState(SideStates.States.Transfer);
                    }
                } else if (frontSide.getCurrentState() == SideStates.States.Transfer) {
                    frontSide.setArmState(SideStates.States.Transfer);
                    backSide.setArmState(SideStates.States.Transfer);
                    if (frontSide.returnColor() == "unknown" && backSide.returnColor() != "unknown") {
                        frontSide.setObserverApproval(true);
                        backSide.setObserverApproval(true);
                        robotState = RobotStates.Unknown;
                    }
                } else frontSide.setArmState(SideStates.States.MTT);
                break;
            case HighJct:
                if (backSide.getCurrentState() == SideStates.States.MTH) {
                    backSide.setArmState(SideStates.States.MTH);
                    if (backSide.getReadinessStatus()) {
                        frontSide.setArmState(SideStates.States.MTI);
                    }
                    if (backSide.getCompletionStatus()) {
                        backSide.setArmState(SideStates.States.Delivery);
                    }
                } else if (backSide.getCurrentState() == SideStates.States.Delivery) {
                    if (backSide.getCompletionStatus()) {
                        robotState = RobotStates.Unknown;
                    }
                } else backSide.setArmState(SideStates.States.MTH);
                break;
            case MedJct:
                if (backSide.getCurrentState() == SideStates.States.MTM) {
                    backSide.setArmState(SideStates.States.MTM);
                    if (backSide.getReadinessStatus()) {
                        frontSide.setArmState(SideStates.States.MTI);
                    }
                    if (backSide.getCompletionStatus()) {
                        backSide.setArmState(SideStates.States.Delivery);
                    }
                } else if (backSide.getCurrentState() == SideStates.States.Delivery) {
                    if (backSide.getCompletionStatus()) {
                        robotState = RobotStates.Unknown;
                    }
                } else backSide.setArmState(SideStates.States.MTM);
                break;
            case LowJct:
                if (backSide.getCurrentState() == SideStates.States.MTL) {
                    backSide.setArmState(SideStates.States.MTL);
                    if (backSide.getReadinessStatus()) {
                        frontSide.setArmState(SideStates.States.MTI);
                    }
                    if (backSide.getCompletionStatus()) {
                        backSide.setArmState(SideStates.States.Delivery);
                    }
                } else if (backSide.getCurrentState() == SideStates.States.Delivery) {
                    if (backSide.getCompletionStatus()) {
                        robotState = RobotStates.Unknown;
                    }
                } else backSide.setArmState(SideStates.States.MTL);
                break;
            case GndJct:
                if (backSide.getCurrentState() == SideStates.States.MTG) {
                    backSide.setArmState(SideStates.States.MTG);
                    if (backSide.getReadinessStatus()) {
                        frontSide.setArmState(SideStates.States.MTI);
                    }
                    if (backSide.getCompletionStatus()) {
                        backSide.setArmState(SideStates.States.Delivery);
                    }
                } else if (backSide.getCurrentState() == SideStates.States.Delivery) {
                    if (backSide.getCompletionStatus()) {
                        robotState = RobotStates.Unknown;
                    }
                } else backSide.setArmState(SideStates.States.MTG);
                break;
            case Unknown:
                frontSide.setArmState(SideStates.States.Unknown);
                backSide.setArmState(SideStates.States.Unknown);
                break;
            default:
                robotState = RobotStates.Unknown;
                break;

        }
    }

    //method that switches front and back between true front and back
    public void changeScoringSide (boolean backScoring) {
        if (backScoring) {
            frontSide = trueFront;
            backSide = trueBack;
        } else {
            backSide = trueFront;
            frontSide = trueBack;
        }
    }
    public RobotStates getRobotState() {return robotState;}
    public SideStates.States getFrontState() {return trueFront.getCurrentState();}
    public SideStates.States getBackState() {return trueBack.getCurrentState();}
    public boolean getFrontRun() {return frontSide.getHasRun();}
    public boolean getBackRun() {return backSide.getHasRun();}
}
