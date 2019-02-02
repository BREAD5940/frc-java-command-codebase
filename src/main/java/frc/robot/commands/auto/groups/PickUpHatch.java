package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConfig;
import frc.robot.commands.auto.actions.DriveDistance;
import frc.robot.commands.subsystems.elevator.SetElevatorHeight;
import frc.robot.commands.subsystems.intake.GrabHatch;
import frc.robot.subsystems.Elevator.ElevatorPresets;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTarget;

/** 
 * runs a series of commands to pick up a hatch from the loading station
 * FIXME test irl
 */
public class PickUpHatch extends CommandGroup{
    public PickUpHatch() {
        /* The plan right now is to lower the elevator, drive to
         a distance from the loading station based on the Lidar, 
         move the elevator up while intaking and once up to a 
        set height start slowly backing up.
        */
        // lifts to center height
        addSequential(new SetElevatorHeight(ElevatorPresets.CARGO_SHIP_HATCH, true)); //cs hatch is same as loading station
        // rams into the loading station (hopefully)
        addSequential(new FollowVisionTarget(0.6, 100, 20)); //FIXME percent frame check
        // grabs the hatch by opening the clamp
        addSequential(new GrabHatch());
        // lifts the hatch out of the brushes //TODO is this necessary?
        addSequential(new SetElevatorHeight(RobotConfig.auto.fieldPositions.cargo_ship_hatch+10, false));
        // moves the robot back slightly
        addSequential(new DriveDistance(-1, 20)); // FIXME check values
        //robot returns to operator control
    }

}