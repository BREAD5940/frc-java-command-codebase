package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.commands.auto.actions.DriveDistance;
import frc.robot.commands.subsystems.elevator.SetElevatorHeight;
import frc.robot.commands.subsystems.intake.OpenClamp;
import frc.robot.subsystems.superstructure.Elevator.ElevatorPresets;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTarget;

/** 
 * runs a series of commands to pick up a hatch from the loading station
 * TODO test irl
 */
public class GrabHatch extends CommandGroup{
    public GrabHatch() {
        /* The plan right now is to lower the elevator, drive to
         a distance from the loading station based on the Lidar, 
         move the elevator up while intaking and once up to a 
        set height start slowly backing up.
        */
        // lifts to center height
        addSequential(Robot.superstructure.moveSuperstructureElevator(ElevatorPresets.CARGO_SHIP_HATCH.getValue())); //cs hatch is same as loading station
        // rams into the loading station (hopefully)
        addSequential(new FollowVisionTarget(0.6, 100, 20)); //TODO percent frame check
        // grabs the hatch by opening the clamp
        addSequential(new OpenClamp());
        // lifts the hatch out of the brushes
        addSequential(Robot.superstructure.moveSuperstructureElevator(RobotConfig.auto.fieldPositions.cargo_ship_hatch+10));
        // moves the robot back slightly
        addSequential(new DriveDistance(-1, 20)); // TODO check values
    }

}