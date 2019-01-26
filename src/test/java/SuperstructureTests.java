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
    SuperstructureState expected1 = new SuperstructureState(gState1);
    SuperstructureState actual1 = SuperstructurePlanner.plan(gState1, currentState);
    System.out.print(expected1.getWristAngle());
    System.out.print(", ");
    System.out.println(actual1.getWristAngle());

    SuperstructureState gState2 = new SuperstructureState(0,WristPos.HATCH,mHeldPiece.NONE);
    SuperstructureState expected2 = gState2; expected2.setWristAngle(Wrist.WristPos.HATCH);
    SuperstructureState actual2 = SuperstructurePlanner.plan(gState2, currentState);
    System.out.print(expected2.getWristAngle());
    System.out.print(", ");
    System.out.println(actual2.getWristAngle());

    SuperstructureState gState3 = new SuperstructureState(0,WristPos.DOWN,mHeldPiece.NONE);
    SuperstructureState expected3 = gState3; expected3.setWristAngle(Wrist.WristPos.CARGO);
    SuperstructureState actual3 = SuperstructurePlanner.plan(gState3, currentState);
    System.out.print(expected3.getWristAngle());
    System.out.print(", ");
    System.out.println(actual3.getWristAngle());

    // heldPiece cargo
    currentState.setHeldPiece(mHeldPiece.CARGO);

    SuperstructureState gState4 = new SuperstructureState(0,WristPos.CARGO,mHeldPiece.CARGO);
    SuperstructureState expected4 = gState4;
    SuperstructureState actual4 = SuperstructurePlanner.plan(gState4, currentState);
    System.out.print(expected4.getWristAngle());
    System.out.print(", ");
    System.out.println(actual4.getWristAngle());

    SuperstructureState gState5 = new SuperstructureState(0,WristPos.HATCH,mHeldPiece.CARGO);
    SuperstructureState expected5 = gState5; expected5.setWristAngle(WristPos.HATCH);
    SuperstructureState actual5 = SuperstructurePlanner.plan(gState5, currentState);
    System.out.print(expected5.getWristAngle());
    System.out.print(", ");
    System.out.println(actual5.getWristAngle());

    SuperstructureState gState6 = new SuperstructureState(0,WristPos.DOWN,mHeldPiece.CARGO);
    SuperstructureState expected6 = gState6; expected6.setWristAngle(WristPos.CARGO);
    SuperstructureState actual6 = SuperstructurePlanner.plan(gState6, currentState);
    System.out.print(expected6.getWristAngle());
    System.out.print(", ");
    System.out.println(actual6.getWristAngle());

    // heldPiece hatch
    currentState.setHeldPiece(mHeldPiece.HATCH);

    SuperstructureState gState7 = new SuperstructureState(0,WristPos.CARGO,mHeldPiece.HATCH);
    SuperstructureState expected7 = gState7;
    SuperstructureState actual7 = SuperstructurePlanner.plan(gState7, currentState);
    System.out.print(expected7.getWristAngle());
    System.out.print(", ");
    System.out.println(actual7.getWristAngle());

    SuperstructureState gState8 = new SuperstructureState(0,WristPos.HATCH,mHeldPiece.HATCH);
    SuperstructureState expected8 = gState7; expected8.setWristAngle(WristPos.HATCH);
    SuperstructureState actual8 = SuperstructurePlanner.plan(gState8, currentState);
    System.out.print(expected8.getWristAngle());
    System.out.print(", ");
    System.out.println(actual8.getWristAngle());

    SuperstructureState gState9 = new SuperstructureState(0,WristPos.DOWN,mHeldPiece.HATCH);
    SuperstructureState expected9 = gState9; expected9.setWristAngle(WristPos.CARGO);
    SuperstructureState actual9 = SuperstructurePlanner.plan(gState9, currentState);
    System.out.print(expected9.getWristAngle());
    System.out.print(", ");
    System.out.println(actual9.getWristAngle()+"\n");
    
    
    assertEquals(expected1.getWristAngle(), actual1.getWristAngle());
    
    assertEquals(expected2.getWristAngle(), actual2.getWristAngle());    
   
    assertEquals(expected3.getWristAngle(), actual3.getWristAngle());    
    
    assertEquals(expected4.getWristAngle(), actual4.getWristAngle());    
    
    assertEquals(expected5.getWristAngle(), actual5.getWristAngle());    
    
    assertEquals(expected6.getWristAngle(), actual6.getWristAngle());    
    
    assertEquals(expected7.getWristAngle(), actual7.getWristAngle());    
    
    assertEquals(expected8.getWristAngle(), actual8.getWristAngle());    
    
    assertEquals(expected9.getWristAngle(), actual9.getWristAngle());    

  }
}