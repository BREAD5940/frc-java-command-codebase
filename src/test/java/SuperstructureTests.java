import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.commands.subsystems.superstructure.wrist.SetWrist;
import frc.robot.lib.SuperstructurePlanner;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.superstructure.Wrist;
import frc.robot.subsystems.superstructure.Wrist.WristPos;

public class SuperstructureTests{
  

  @Test
  public void testPresetWrists(){
    SuperstructurePlanner planner = new SuperstructurePlanner();
    SuperstructureState currentState = new SuperstructureState(0,WristPos.CARGO,mHeldPiece.NONE);

    // No heldPiece
    SuperstructureState gState1 = new SuperstructureState(0,WristPos.CARGO,mHeldPiece.NONE);
    CommandGroup expected1 = new CommandGroup();
    CommandGroup actual1 = planner.plan(gState1, currentState);

    SuperstructureState gState2 = new SuperstructureState(0,WristPos.HATCH,mHeldPiece.NONE);
    CommandGroup expected2 = new CommandGroup(); expected2.addSequential(new SetWrist(Wrist.WristPos.HATCH));
    CommandGroup actual2 = planner.plan(gState2, currentState);

    SuperstructureState gState3 = new SuperstructureState(0,WristPos.DOWN,mHeldPiece.NONE);
    CommandGroup expected3 = new CommandGroup(); expected3.addSequential(new SetWrist(Wrist.WristPos.CARGO));
    CommandGroup actual3 = planner.plan(gState3, currentState);

    // heldPiece cargo
    currentState.setHeldPiece(mHeldPiece.CARGO);

    SuperstructureState gState4 = new SuperstructureState(0,WristPos.CARGO,mHeldPiece.CARGO);
    CommandGroup expected4 = new CommandGroup();
    CommandGroup actual4 = planner.plan(gState4, currentState);

    SuperstructureState gState5 = new SuperstructureState(0,WristPos.HATCH,mHeldPiece.CARGO);
    CommandGroup expected5 = new CommandGroup(); expected5.addSequential(new SetWrist(WristPos.HATCH));
    CommandGroup actual5 = planner.plan(gState5, currentState);

    SuperstructureState gState6 = new SuperstructureState(0,WristPos.DOWN,mHeldPiece.CARGO);
    CommandGroup expected6 = new CommandGroup(); expected6.addSequential(new SetWrist(WristPos.CARGO));
    CommandGroup actual6 = planner.plan(gState6, currentState);

    // heldPiece hatch
    currentState.setHeldPiece(mHeldPiece.HATCH);

    SuperstructureState gState7 = new SuperstructureState(0,WristPos.CARGO,mHeldPiece.HATCH);
    CommandGroup expected7 = new CommandGroup();
    CommandGroup actual7 = planner.plan(gState7, currentState);

    SuperstructureState gState8 = new SuperstructureState(0,WristPos.HATCH,mHeldPiece.HATCH);
    CommandGroup expected8 = new CommandGroup(); expected8.addSequential(new SetWrist(WristPos.HATCH));
    CommandGroup actual8 = planner.plan(gState8, currentState);

    SuperstructureState gState9 = new SuperstructureState(0,WristPos.DOWN,mHeldPiece.HATCH);
    CommandGroup expected9 = new CommandGroup(); expected9.addSequential(new SetWrist(WristPos.CARGO));
    CommandGroup actual9 = planner.plan(gState9, currentState);
    

    assertEquals(expected1, actual1);
    assertEquals(expected2, actual2);
    assertEquals(expected3, actual3);
    assertEquals(expected4, actual4);
    assertEquals(expected5, actual5);
    assertEquals(expected6, actual6);
    assertEquals(expected7, actual7);
    assertEquals(expected8, actual8);
    assertEquals(expected9, actual9);

  }
}