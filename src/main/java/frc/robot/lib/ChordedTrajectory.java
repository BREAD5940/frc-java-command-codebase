package frc.robot.lib;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import frc.robot.dhdMap;
import frc.robot.commands.auto.AutoCombo;
import frc.robot.commands.auto.Trajectories;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.xboxmap.Buttons;

public class ChordedTrajectory{
  private static String startKey, endKey;
  public ChordedTrajectory(){}

  private static void getKeys(){
    String sbase = "hab";
    char sside='M', sloc='1';
    if(dhdMap.Start.HAB.get()){
      sbase="hab";
    }else if(dhdMap.Start.LOAD.get()){
      sbase="loading";
    }else if(dhdMap.Start.DEPOT.get()){
      sbase="depot";
    }else if (dhdMap.Start.CARGO.get()){
      sbase="cargo";
    }else if(dhdMap.Start.ROCKET.get()){
      sbase="rocket";
    }

    if(dhdMap.Start.L.get()){
      sside='L';
    }else if(dhdMap.Start.M.get()){
      sside='M';
    }else if (dhdMap.Start.R.get()){
      sside='R';
    }

    if(dhdMap.Start.ONE.get()){
      sloc='1';
    }else if(dhdMap.Start.TWO.get()){
      sloc='2';
    }else if(dhdMap.Start.THREE.get()){
      sloc='3';
    }

    startKey = sbase+sside+sloc;

    String ebase="hab";
    char eside='M', eloc='1';
    if(dhdMap.End.HAB.get()){
      ebase="hab";
    }else if(dhdMap.End.LOAD.get()){
      ebase="loading";
    }else if(dhdMap.End.DEPOT.get()){
      ebase="depot";
    }else if (dhdMap.End.CARGO.get()){
      ebase="cargo";
    }else if(dhdMap.End.ROCKET.get()){
      ebase="rocket";
    }

    if(dhdMap.End.L.get()){
      eside='L';
    }else if(dhdMap.End.M.get()){
      eside='M';
    }else if (dhdMap.End.R.get()){
      eside='R';
    }

    if(dhdMap.End.ONE.get()){
      eloc='1';
    }else if(dhdMap.End.TWO.get()){
      eloc='2';
    }else if(dhdMap.End.THREE.get()){
      eloc='3';
    }

    endKey = ebase+eside+eloc;
  }

  public static TimedTrajectory<Pose2dWithCurvature> getTraject(Gear gear){
    getKeys();
    if(gear==Gear.HIGH){
      return Trajectories.generatedHGTrajectories.get(startKey+" to "+endKey);
    }else{
      return Trajectories.generatedLGTrajectories.get(startKey+" to "+endKey);
    }
  }
}