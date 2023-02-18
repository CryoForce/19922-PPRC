package org.firstinspires.ftc.teamcode;

public class Observer {
    private SideStates.States robotState;
    private SideStates frontSide;
    private SideStates backSide;
    private HWC bronto;

    public Observer (HWC initBronto) {
        this.bronto = initBronto;
        frontSide = new SideStates(bronto,
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
        backSide = new SideStates(bronto,
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
    }
    public void setCycleState (SideStates.States newState) {
        robotState = newState;
        switch (robotState) {
            case Intake:
                if (frontSide.getCurrentState() == SideStates.States.MTI) {
                    frontSide.setArmState(SideStates.States.MTI);
                    if (frontSide.getReadinessStatus()) {
                        backSide.setArmState(SideStates.States.MTT);
                    }
                } else if (frontSide.getCurrentState() == SideStates.States.Intake){
                    frontSide.setArmState(SideStates.States.Intake);
                    backSide.setArmState(SideStates.States.MTT);
                    if (frontSide.getCompletionStatus()) {
                        robotState = SideStates.States.Unknown;
                    }
                }
                break;
        }
    }
    public void optimizeArmStates() {
        if (frontSide.getCurrentState() == SideStates.States.MTT
                && backSide.getCurrentState() == SideStates.States.MTT) {

        }
    }
}
