// import static org.junit.jupiter.api.Assertions.assertEquals;

// import java.util.ArrayList;
// import java.util.Arrays;

// import org.junit.jupiter.api.Test;

// public class SuperstructureTests {

//   @Test
//   public void testWrists(){
//     SuperstructurePlanner planner = new SuperstructurePlanner();
//     SuperstructureState currentState = new SuperstructureState(0,iPosition.CARGO_GRAB,HeldPiece.NONE);
//     ArrayList<SuperstructureState> goalStates = new ArrayList<SuperstructureState>(Arrays.asList(
//               new SuperstructureState(0,iPosition.CARGO_GRAB,HeldPiece.NONE),
//               new SuperstructureState(0,iPosition.HATCH,HeldPiece.NONE),
//               new SuperstructureState(0,iPosition.CARGO_DOWN,HeldPiece.NONE),
//               new SuperstructureState(0,new IntakeAngle(10,10),HeldPiece.NONE),
//               new SuperstructureState(0,iPosition.CARGO_GRAB,HeldPiece.CARGO)));
//     ArrayList<SuperstructureState> correctEndStates = new ArrayList<SuperstructureState>(Arrays.asList(
//               new SuperstructureState(0,iPosition.CARGO_GRAB,HeldPiece.NONE),
//               new SuperstructureState(0,iPosition.HATCH,HeldPiece.NONE),
//               new SuperstructureState(0,iPosition.CARGO_GRAB,HeldPiece.NONE),
//               new SuperstructureState(0,new IntakeAngle(10,10),HeldPiece.NONE),
//               new SuperstructureState(0,iPosition.CARGO_GRAB,HeldPiece.NONE)));
//     ArrayList<SuperstructureState> resultingStates=new ArrayList<SuperstructureState>();

//     for(int i=0; i<goalStates.size(); i++){
//       resultingStates.add(i,planner.getPlannedState(goalStates.get(i),currentState));
//       System.out.print(Integer.valueOf(i)+": ");
//       System.out.print(correctEndStates.get(i).getAngle());
//       System.out.print(", ");
//       System.out.println(resultingStates.get(i).getAngle());
//       assertEquals(correctEndStates.get(i).getAngle(), resultingStates.get(i).getAngle());
//     }
//     System.out.println();
//   }

//   @Test
//   public void testElevator(){
//     SuperstructurePlanner planner = new SuperstructurePlanner();
//     SuperstructureState currentState = new SuperstructureState(0,iPosition.CARGO_GRAB,HeldPiece.NONE);
//     ArrayList<SuperstructureState> goalStates = new ArrayList<SuperstructureState>(Arrays.asList(
//               new SuperstructureState(10,iPosition.CARGO_GRAB,HeldPiece.NONE),
//               new SuperstructureState(30,iPosition.CARGO_GRAB,HeldPiece.NONE),
//               new SuperstructureState(80,iPosition.CARGO_GRAB,HeldPiece.NONE)));
//     ArrayList<SuperstructureState> correctEndStates = new ArrayList<SuperstructureState>(Arrays.asList(
//               new SuperstructureState(10,iPosition.CARGO_GRAB,HeldPiece.NONE),
//               new SuperstructureState(30,iPosition.CARGO_GRAB,HeldPiece.NONE),
//               new SuperstructureState(70,iPosition.CARGO_GRAB,HeldPiece.NONE)));
//     ArrayList<SuperstructureState> resultingStates=new ArrayList<SuperstructureState>();

//     for(int i=0; i<goalStates.size(); i++){
//       resultingStates.add(i,planner.getPlannedState(goalStates.get(i),currentState));
//       System.out.print(Integer.valueOf(i)+": ");
//       System.out.print(correctEndStates.get(i).getElevatorHeight());
//       System.out.print(", "); 
//       System.out.println(resultingStates.get(i).getElevatorHeight());
//       assertEquals(correctEndStates.get(i).getElevatorHeight(), resultingStates.get(i).getElevatorHeight()); 
//     }
//     System.out.println();
//   }

//   @Test
//   public void bigScaryComboTests(){
//     SuperstructurePlanner planner = new SuperstructurePlanner();
//     SuperStructureState currentState = new SuperStructureState(0,iPosition.CARGO_GRAB,HeldPiece.NONE);
//     ArrayList<SuperStructureState> goalStates = new ArrayList<SuperStructureState>(Arrays.asList(
//               new SuperStructureState(10,iPosition.HATCH,HeldPiece.NONE),
//               new SuperStructureState(30,iPosition.HATCH,HeldPiece.NONE),
//               new SuperStructureState(30,new IntakeAngle(10, 10),HeldPiece.NONE)));
//     ArrayList<SuperstructureState> correctEndStates = new ArrayList<SuperStructureState>(Arrays.asList(
//               new SuperStructureState(10,iPosition.HATCH,HeldPiece.NONE),
//               new SuperStructureState(30,iPosition.HATCH,HeldPiece.NONE),
//               new SuperStructureState(30,new IntakeAngle(10,10),HeldPiece.NONE)));
//     ArrayList<SuperStructureState> resultingStates=new ArrayList<SuperStructureState>();

//     for(int i=0; i<goalStates.size(); i++){
//       resultingStates.add(i,planner.getPlannedState(goalStates.get(i),currentState));

//       System.out.print(Integer.valueOf(i)+" heights: ");
//       System.out.print(correctEndStates.get(i).getElevatorHeight());
//       System.out.print(", "); 
//       System.out.println(resultingStates.get(i).getElevatorHeight());
//       assertEquals(correctEndStates.get(i).getElevatorHeight(), resultingStates.get(i).getElevatorHeight()); 

//       System.out.print(Integer.valueOf(i)+" angles: ");
//       resultingStates.add(i,planner.getPlannedState(goalStates.get(i),currentState));
//       System.out.print(Integer.valueOf(i)+": ");
//       System.out.print(correctEndStates.get(i).getAngle());
//       System.out.print(", ");
//       System.out.println(resultingStates.get(i).getAngle());
//       assertEquals(correctEndStates.get(i).getAngle(), resultingStates.get(i).getAngle());
//     }
//     System.out.println();
//   }
// }
