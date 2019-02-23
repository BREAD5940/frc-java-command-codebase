package frc.robot.planners;


import java.util.LinkedList;
import java.util.Optional;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.SuperStructureConstants;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;

public class SuperstructurePlannerTheSecond {
    private boolean mUpwardsSubcommandEnabled = true;

    class FakeCommand {
        public FakeCommand(SuperStructureState endState) {
            mEndState = endState;
        }

        public SuperStructureState mEndState;
        public Length mHeightThreshold = LengthKt.getInch(1.0);
        public RoundRotation2d mElbowThreshold = RoundRotation2d.getDegree(5.0);

        public boolean isFinished(SuperStructureState currentState) {
						return Math.abs(mEndState.getElevatorHeight().getInch()-currentState.getElevatorHeight().getInch())<= mHeightThreshold.getInch()
									&& Math.abs(mEndState.getElbow().angle.getDegree()-currentState.getElbow().angle.getDegree())<= mElbowThreshold.getDegree();
        }
    }

    class WaitForElbowSafeSubcommand extends FakeCommand {
        public WaitForElbowSafeSubcommand(SuperStructureState endState) {
						super(endState);
						mElbowThreshold = mElbowThreshold.plus(RoundRotation2d.getDegree(Math.max(0.0, mEndState.getElbow().angle.getDegree() - 
						SuperStructureConstants.Elbow.kClearFirstStageMinElbowAngle.getDegree())));
        }

        @Override
        public boolean isFinished(SuperStructureState currentState) {
						return Math.abs(mEndState.getElevatorHeight().getInch()-currentState.getElevatorHeight().getInch())<= Double.POSITIVE_INFINITY
						&& Math.abs(mEndState.getElbow().angle.getDegree()-currentState.getElbow().angle.getDegree())<= mElbowThreshold.getDegree();
        }
    }

    class WaitForElevatorSafeSubcommand extends FakeCommand {
        public WaitForElevatorSafeSubcommand(SuperStructureState endState, SuperStructureState currentState) {
            super(endState);
            if (endState.getElevatorHeight().getInch() >= currentState.getElevatorHeight().getInch()) {
								mHeightThreshold = mHeightThreshold.plus(LengthKt.getInch(Math.max(0.0, mEndState.getElevatorHeight().getInch() - 
								SuperStructureConstants.Elevator.kClearFirstStageMaxHeight.getInch())));
            } else {
                mHeightThreshold = mHeightThreshold.plus(LengthKt.getInch(Math.max(0.0, SuperStructureConstants.Elevator.kClearFirstStageMaxHeight.getInch()
                        - mEndState.getElevatorHeight().getInch())));
            }
        }

        @Override
        public boolean isFinished(SuperStructureState currentState) {
						return Math.abs(mEndState.getElevatorHeight().getInch()-currentState.getElevatorHeight().getInch())<= mHeightThreshold.getInch()
						&& Math.abs(mEndState.getElbow().angle.getDegree()-currentState.getElbow().angle.getDegree())<= Double.POSITIVE_INFINITY;
        }
    }

    class WaitForElevatorApproachingSubcommand extends FakeCommand {
        public WaitForElevatorApproachingSubcommand(SuperStructureState endState) {
            super(endState);
            mHeightThreshold = SuperStructureConstants.Elevator.kElevatorApproachingThreshold;
        }

        @Override
        public boolean isFinished(SuperStructureState currentState) {
						return Math.abs(mEndState.getElevatorHeight().getInch()-currentState.getElevatorHeight().getInch())<= mHeightThreshold.getInch()
						&& Math.abs(mEndState.getElbow().angle.getDegree()-currentState.getElbow().angle.getDegree())<= Double.POSITIVE_INFINITY;
        }
    }

    class WaitForFinalSetpointSubcommand extends FakeCommand {
        public WaitForFinalSetpointSubcommand(SuperStructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperStructureState currentState) {
            return currentState.elevatorSentLastTrajectory && currentState.elbowSentLastTrajectory;
        }
    }

    protected SuperStructureState mCommandedState = new SuperStructureState();
    protected SuperStructureState mIntermediateCommandState = new SuperStructureState();
    protected LinkedList<FakeCommand> mCommandQueue = new LinkedList<>();
    protected Optional<FakeCommand> mCurrentCommand = Optional.empty();

    public synchronized boolean setDesiredState(SuperStructureState desiredStateIn, SuperStructureState currentState) {
        SuperStructureState desiredState = new SuperStructureState(desiredStateIn);

        // Limit illegal inputs.
        desiredState.getElbow().angle = RoundRotation2d.getDegree(Util.limit(desiredState.getElbow().angle.getDegree(), SuperStructureConstants.Elbow.kElbowMin.getDegree(),
                SuperStructureConstants.Elbow.kElbowMax.getDegree()));
        desiredState.getElevator().setHeight(LengthKt.getInch(Util.limit(desiredState.getElevatorHeight().getInch(), SuperStructureConstants.Elevator.bottom.getInch(),
                SuperStructureConstants.Elevator.top.getInch())));


        // Everything beyond this is probably do-able; clear queue
        mCommandQueue.clear();

				final boolean longUpwardsMove = desiredState.getElevatorHeight().getInch() - currentState.getElevatorHeight().getInch() > 
				SuperStructureConstants.Elevator.kElevatorLongRaiseDistance.getInch();
				final RoundRotation2d firstElbowAngle = longUpwardsMove ? RoundRotation2d.getDegree(Math.min(desiredState.getElbow().angle.getDegree(), 
									SuperStructureConstants.Elbow.kStowedAngle.getDegree())) : desiredState.getElbow().angle;

        if (currentState.getElbow().angle.getDegree() < SuperStructureConstants.Elbow.kClearFirstStageMinElbowAngle.getDegree() && desiredState.getElevatorHeight().getInch() >
                SuperStructureConstants.Elevator.kClearFirstStageMaxHeight.getInch()) {
            // PRECONDITION: Elbow is unsafe, want to go high
            // mCommandQueue.add(new WaitForElbowSafeSubcommand(new SuperStructureState(SuperStructureConstants
            // .Elevator.kClearFirstStageMaxHeight.getInch(), Math.max(SuperStructureConstants
            //        .kClearFirstStageMinElbowAngle, firstElbowAngle), true)));
            // POSTCONDITION: Elbow is safe (either at desired getElbow().angle.getDegree(), or the cruise getElbow().angle.getDegree()), elevator is as close as
            // possible to goal.
        } else if (desiredState.getElbow().angle.getDegree() < SuperStructureConstants.Elbow.kClearFirstStageMinElbowAngle.getDegree() && currentState.getElevatorHeight().getInch()
                > SuperStructureConstants.Elevator.kClearFirstStageMaxHeight.getInch()) {
            // PRECONDITION: Elbow is safe, want to go low.
//            mCommandQueue.add(new WaitForElevatorSafeSubcommand(new SuperStructureState(desiredState.getElevatorHeight().getInch(),
//                    SuperStructureConstants
//                    .kClearFirstStageMinElbowAngle, true), currentState));
            // POSTCONDITION: elevator is safe, Elbow is as close as possible to goal.
        }

        if (longUpwardsMove) {
            // PRECONDITION: Elbow is safe, we are moving upwards.
            if (mUpwardsSubcommandEnabled) {
                mCommandQueue.add(new WaitForElevatorApproachingSubcommand(new SuperStructureState(desiredState.getElevator(),
                        new RotatingArmState(firstElbowAngle), new RotatingArmState())));
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
        return mCurrentCommand.isPresent() && mCommandQueue.isEmpty() && currentState.elbowSentLastTrajectory &&
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
            FakeCommand subCommand = mCurrentCommand.get();
            mIntermediateCommandState = subCommand.mEndState;
            if (subCommand.isFinished(currentState) && !mCommandQueue.isEmpty()) {
                // Let the current command persist until there is something in the queue. or not. desired outcome
                // unclear.
                mCurrentCommand = Optional.empty();
            }
        } else {
            mIntermediateCommandState = currentState;
        }

        mCommandedState.getElbow().angle = RoundRotation2d.getDegree(Util.limit(mIntermediateCommandState.getElbow().angle.getDegree(), SuperStructureConstants.Elbow.kElbowMin.getDegree(),
                SuperStructureConstants.Elbow.kElbowMax.getDegree()));
        mCommandedState.getElevator().setHeight(LengthKt.getInch(Util.limit(mIntermediateCommandState.getElevatorHeight().getInch(), SuperStructureConstants
                .Elevator.bottom.getInch(), SuperStructureConstants.Elevator.top.getInch())));

        return mCommandedState;
    }
}
