package frc.robot.planners;


import java.util.LinkedList;
import java.util.Optional;

import frc.robot.SuperStructureConstants;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.SuperStructureState;

public class SuperstructurePlannerTheSecond {
    private boolean mUpwardsSubcommandEnabled = true;

    class SubCommand {
        public SubCommand(SuperStructureState endState) {
            mEndState = endState;
        }

        public SuperStructureState mEndState;
        public double mHeightThreshold = 1.0;
        public double mWristThreshold = 5.0;

        public boolean isFinished(SuperStructureState currentState) {
            return mEndState.isInRange(currentState, mHeightThreshold, mWristThreshold);
        }
    }

    class WaitForWristSafeSubcommand extends SubCommand {
        public WaitForWristSafeSubcommand(SuperStructureState endState) {
			super(endState);
            mWristThreshold = mWristThreshold + Math.max(0.0, mEndState.getWrist().angle.getDegree() - SuperStructureConstants
                    .kClearFirstStageMinWristAngle);
        }

        @Override
        public boolean isFinished(SuperStructureState currentState) {
            return mEndState.isInRange(currentState, Double.POSITIVE_INFINITY, mWristThreshold);
        }
    }

    class WaitForElevatorSafeSubcommand extends SubCommand {
        public WaitForElevatorSafeSubcommand(SuperStructureState endState, SuperStructureState currentState) {
            super(endState);
            if (endState.getElevatorHeight().getInch() >= currentState.getElevatorHeight().getInch()) {
                mHeightThreshold = mHeightThreshold + Math.max(0.0, mEndState.getElevatorHeight().getInch() - SuperStructureConstants
                        .kClearFirstStageMaxHeight);
            } else {
                mHeightThreshold = mHeightThreshold + Math.max(0.0, SuperStructureConstants.kClearFirstStageMaxHeight
                        - mEndState.getElevatorHeight().getInch());
            }
        }

        @Override
        public boolean isFinished(SuperStructureState currentState) {
            return mEndState.isInRange(currentState, mHeightThreshold, Double.POSITIVE_INFINITY);
        }
    }

    class WaitForElevatorApproachingSubcommand extends SubCommand {
        public WaitForElevatorApproachingSubcommand(SuperStructureState endState) {
            super(endState);
            mHeightThreshold = SuperStructureConstants.kElevatorApproachingThreshold;
        }

        @Override
        public boolean isFinished(SuperStructureState currentState) {
            return mEndState.isInRange(currentState, mHeightThreshold, Double.POSITIVE_INFINITY);
        }
    }

    class WaitForFinalSetpointSubcommand extends SubCommand {
        public WaitForFinalSetpointSubcommand(SuperStructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperStructureState currentState) {
            return currentState.elevatorSentLastTrajectory && currentState.wristSentLastTrajectory;
        }
    }

    protected SuperStructureState mCommandedState = new SuperStructureState();
    protected SuperStructureState mIntermediateCommandState = new SuperStructureState();
    protected LinkedList<SubCommand> mCommandQueue = new LinkedList<>();
    protected Optional<SubCommand> mCurrentCommand = Optional.empty();

    public synchronized boolean setDesiredState(SuperStructureState desiredStateIn, SuperStructureState currentState) {
        SuperStructureState desiredState = new SuperStructureState(desiredStateIn);

        // Limit illegal inputs.
        desiredState.getWrist().angle = RoundRotation2d.getDouble(Util.limit(desiredState.getWrist().angle.getDegree(), SuperStructureConstants.Wrist.kWristMin.getDegree(),
                SuperStructureConstants.Wrist.kWristMax.getInch()));
        desiredState.getElevator().setHeight(Util.limit(desiredState.getElevatorHeight().getInch(), SuperStructureConstants.Elevator.bottom.getInch(),
                SuperStructureConstants.Elevator.top.getInch()));


        // Everything beyond this is probably do-able; clear queue
        mCommandQueue.clear();

        final boolean longUpwardsMove = desiredState.getElevatorHeight().getInch() - currentState.getElevatorHeight().getInch() > SuperStructureConstants
                .kElevatorLongRaiseDistance;
        final double firstWristAngle = longUpwardsMove ? Math.min(desiredState.getWrist().angle.getDegree(), SuperStructureConstants
                .kStowedAngle) : desiredState.getWrist().angle.getDegree();

        if (currentState.getWrist().angle.getDegree() < SuperStructureConstants.kClearFirstStageMinWristAngle && desiredState.getElevatorHeight().getInch() >
                SuperStructureConstants.kClearFirstStageMaxHeight) {
            // PRECONDITION: wrist is unsafe, want to go high
            // mCommandQueue.add(new WaitForWristSafeSubcommand(new SuperStructureState(SuperStructureConstants
            // .kClearFirstStageMaxHeight, Math.max(SuperStructureConstants
            //        .kClearFirstStageMinWristAngle, firstWristAngle), true)));
            // POSTCONDITION: wrist is safe (either at desired getWrist().angle.getDegree(), or the cruise getWrist().angle.getDegree()), elevator is as close as
            // possible to goal.
        } else if (desiredState.getWrist().angle.getDegree() < SuperStructureConstants.kClearFirstStageMinWristAngle && currentState.getElevatorHeight().getInch()
                > SuperStructureConstants.kClearFirstStageMaxHeight) {
            // PRECONDITION: wrist is safe, want to go low.
//            mCommandQueue.add(new WaitForElevatorSafeSubcommand(new SuperStructureState(desiredState.getElevatorHeight().getInch(),
//                    SuperStructureConstants
//                    .kClearFirstStageMinWristAngle, true), currentState));
            // POSTCONDITION: elevator is safe, wrist is as close as possible to goal.
        }

        if (longUpwardsMove) {
            // PRECONDITION: wrist is safe, we are moving upwards.
            if (mUpwardsSubcommandEnabled) {
                mCommandQueue.add(new WaitForElevatorApproachingSubcommand(new SuperStructureState(desiredState.getElevator(),
                        firstWristAngle)));
            }
            // POSTCONDITION: elevator is approaching final goal.
        }

        // Go to the goal.
        mCommandQueue.add(new WaitForFinalSetpointSubcommand(desiredState));

        // Reset current command to start executing on next iteration
        mCurrentCommand = Optional.empty();

        return true; // this is a legal move
    }

    void reset(SuperStructureState currentState) {
        mIntermediateCommandState = currentState;
        mCommandQueue.clear();
        mCurrentCommand = Optional.empty();
    }

    public boolean isFinished(SuperStructureState currentState) {
        return mCurrentCommand.isPresent() && mCommandQueue.isEmpty() && currentState.wristSentLastTrajectory &&
                currentState.elevatorSentLastTrajectory;
    }

    public synchronized void setUpwardsSubcommandEnable(boolean enabled) {
        mUpwardsSubcommandEnabled = enabled;
    }

    public SuperStructureState update(SuperStructureState currentState) {
        if (!mCurrentCommand.isPresent() && !mCommandQueue.isEmpty()) {
            mCurrentCommand = Optional.of(mCommandQueue.remove());
        }

        if (mCurrentCommand.isPresent()) {
            SubCommand subCommand = mCurrentCommand.get();
            mIntermediateCommandState = subCommand.mEndState;
            if (subCommand.isFinished(currentState) && !mCommandQueue.isEmpty()) {
                // Let the current command persist until there is something in the queue. or not. desired outcome
                // unclear.
                mCurrentCommand = Optional.empty();
            }
        } else {
            mIntermediateCommandState = currentState;
        }

        mCommandedState.getWrist().angle = Util.limit(mIntermediateCommandState.getWrist().angle.getDegree(), SuperStructureConstants.Wrist.kWristMin.getDegree(),
                SuperStructureConstants.Wrist.kWristMax.getDegree());
        mCommandedState.getElevator().setHeight(Util.limit(mIntermediateCommandState.getElevatorHeight().getInch(), SuperStructureConstants
                .Elevator.bottom.getInch(), SuperStructureConstants.Elevator.top.getInch()));

        return mCommandedState;
    }
}
