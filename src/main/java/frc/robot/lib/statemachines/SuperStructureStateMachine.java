package frc.robot.lib.statemachines;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.SuperStructureConstants;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.planners.SuperstructurePlannerTheSecond;
import frc.robot.states.DumbCommandyBoi;
import frc.robot.states.SuperStructureState;

public class SuperStructureStateMachine {
    public enum WantedAction {
        IDLE,
        GO_TO_POSITION,
        WANT_MANUAL,
    }

    public enum SystemState {
        HOLDING_POSITION,
        MOVING_TO_POSITION,
        MANUAL
    }

    private SystemState mSystemState = SystemState.HOLDING_POSITION;

    private SuperStructureState mCommandedState = new SuperStructureState();
    private SuperStructureState mDesiredEndState = new SuperStructureState();

    private SuperstructurePlannerTheSecond mPlanner = new SuperstructurePlannerTheSecond();

    private Length mScoringHeight = SuperStructureConstants.Elevator.bottom;
    private RoundRotation2d mScoringAngle = SuperStructureConstants.Elbow.kStowedAngle;

    private double mOpenLoopPower = 0.0;
    private boolean mManualWantsLowGear = false;
    private Length mMaxHeight = SuperStructureConstants.Elevator.top;

    private DumbCommandyBoi mCommand = new DumbCommandyBoi();

    public synchronized void resetManual() {
        mOpenLoopPower = 0.0;
        mManualWantsLowGear = false;
    }

    public synchronized void setMaxHeight(Length height) {
        mMaxHeight = height;
    }

    public synchronized void setManualWantsLowGear(boolean wantsLowGear) {
        mManualWantsLowGear = wantsLowGear;
    }

    public synchronized void setOpenLoopPower(double power) {
        mOpenLoopPower = power;
    }

    public synchronized void setScoringHeight(Length inches) {
        mScoringHeight = inches;
    }

    public synchronized Length getScoringHeight() {
        return mScoringHeight;
    }

    public synchronized void setScoringAngle(RoundRotation2d angle) {
        mScoringAngle = angle;
    }

    public synchronized RoundRotation2d getScoringAngle() {
        return mScoringAngle;
    }

    public synchronized void jogElevator(Length relative_inches) {
        mScoringHeight = mScoringHeight.plus(relative_inches);
        mScoringHeight = LengthKt.getInch(Math.min(mScoringHeight.getInch(), mMaxHeight.getInch()));
        mScoringHeight = LengthKt.getInch(Math.max(mScoringHeight.getInch(), SuperStructureConstants.Elevator.bottom.getInch()));
    }

    public synchronized void jogWrist(RoundRotation2d relative_degrees) {
        mScoringAngle = mScoringAngle.plus(relative_degrees);
        mScoringAngle = RoundRotation2d.getDegree(Math.min(mScoringAngle.getDegree(), SuperStructureConstants.Elbow.kElbowMax.getDegree()));
        mScoringAngle = RoundRotation2d.getDegree(Math.max(mScoringAngle.getDegree(), SuperStructureConstants.Elbow.kElbowMin.getDegree()));
    }

    public synchronized boolean scoringPositionChanged() {
        return !Util.epsilonEquals(mDesiredEndState.getElbow().angle.getDegree(), mScoringAngle.getDegree()) ||
                !Util.epsilonEquals(mDesiredEndState.getElevatorHeight().getInch(), mScoringHeight.getInch());
    }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    public synchronized void setUpwardsSubcommandEnable(boolean enabled) {
        mPlanner.setUpwardsSubcommandEnable(enabled);
    }

    public synchronized DumbCommandyBoi update(double timestamp, WantedAction wantedAction,
                                                     SuperStructureState currentState) {
        synchronized (SuperStructureStateMachine.this) {
            SystemState newState;

            // Handle state transitions
            switch (mSystemState) {
                case HOLDING_POSITION:
                    newState = handleHoldingPositionTransitions(wantedAction, currentState);
                    break;
                case MOVING_TO_POSITION:
                    newState = handleMovingToPositionTransitions(wantedAction, currentState);
                    break;
                case MANUAL:
                    newState = handleManualTransitions(wantedAction, currentState);
                    break;
                default:
                    System.out.println("Unexpected superstructure system state: " + mSystemState);
                    newState = mSystemState;
                    break;
            }

            if (newState != mSystemState) {
                System.out.println(timestamp + ": Superstructure changed state: " + mSystemState + " -> " + newState);
                mSystemState = newState;
            }

            // Pump elevator planner only if not jogging.
            if (!mCommand.openLoopElevator) {
                mCommandedState = mPlanner.update(currentState);
                mCommand.height = LengthKt.getInch(Math.min(mCommandedState.getElevatorHeight().getInch(), mMaxHeight.getInch()));
                mCommand.wristAngle = mCommandedState.getElbow().angle;
            }

            // Handle state outputs
            switch (mSystemState) {
                case HOLDING_POSITION:
                    getHoldingPositionCommandedState();
                    break;
                case MOVING_TO_POSITION:
                    getMovingToPositionCommandedState();
                    break;
                case MANUAL:
                    getManualCommandedState();
                    break;
                default:
                    System.out.println("Unexpected superstructure state output state: " + mSystemState);
                    break;
            }

            return mCommand;
        }
    }

    private void updateMotionPlannerDesired(SuperStructureState currentState) {
        mDesiredEndState.getElbow().angle = mScoringAngle;
        mDesiredEndState.getElevator().setHeight(mScoringHeight);

        System.out.println("Setting motion planner to height: " + mDesiredEndState.getElevatorHeight().toString()
                + " angle: " + mDesiredEndState.getElbow().angle.toString());

        // Push into elevator planner.
        if (!mPlanner.setDesiredState(mDesiredEndState, currentState)) {
            System.out.println("Unable to set elevator planner!");
        }

        mScoringAngle = mDesiredEndState.getElbow().angle;
        mScoringHeight = mDesiredEndState.getElevatorHeight();
    }

    private SystemState handleDefaultTransitions(WantedAction wantedAction, SuperStructureState currentState) {
        if (wantedAction == WantedAction.GO_TO_POSITION) {
            if (scoringPositionChanged()) {
                updateMotionPlannerDesired(currentState);
            } else if (mPlanner.isFinished(currentState)) {
                return SystemState.HOLDING_POSITION;
            }
            return SystemState.MOVING_TO_POSITION;
        } else if (wantedAction == WantedAction.WANT_MANUAL) {
            return SystemState.MANUAL;
        } else {
            if (mSystemState == SystemState.MOVING_TO_POSITION && !mPlanner.isFinished(currentState)) {
                return SystemState.MOVING_TO_POSITION;
            } else {
                return SystemState.HOLDING_POSITION;
            }
        }
    }

    // HOLDING_POSITION
    private SystemState handleHoldingPositionTransitions(WantedAction wantedAction,
                                                         SuperStructureState currentState) {
        return handleDefaultTransitions(wantedAction, currentState);
    }

    private void getHoldingPositionCommandedState() {
        mCommand.elevatorLowGear = false;
        mCommand.openLoopElevator = false;
    }

    // MOVING_TO_POSITION
    private SystemState handleMovingToPositionTransitions(WantedAction wantedAction,
                                                          SuperStructureState currentState) {

        return handleDefaultTransitions(wantedAction, currentState);
    }

    private void getMovingToPositionCommandedState() {
        mCommand.elevatorLowGear = false;
        mCommand.openLoopElevator = false;
    }

    // MANUAL
    private SystemState handleManualTransitions(WantedAction wantedAction,
                                                SuperStructureState currentState) {
        if (wantedAction != WantedAction.WANT_MANUAL) {
            // Freeze height.
            mScoringAngle = currentState.getElbow().angle;
            mScoringHeight = currentState.getElevatorHeight();
            return handleDefaultTransitions(WantedAction.GO_TO_POSITION, currentState);
        }
        return handleDefaultTransitions(wantedAction, currentState);
    }

    private void getManualCommandedState() {
        mCommand.elevatorLowGear = mManualWantsLowGear;
        mCommand.wristAngle = SuperStructureConstants.Elbow.kElbowMin;
        mCommand.openLoopElevator = true;
        mCommand.openLoopElevatorPercent = mOpenLoopPower;
    }
}
