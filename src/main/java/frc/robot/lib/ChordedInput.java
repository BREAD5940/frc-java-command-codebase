package frc.robot.lib;

import java.util.List;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.dhdMap;
import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;

public class ChordedInput{

	public static GoalType getGT(){
		if(dhdMap.Motion.GRAB_CARGO.get()){
			return GoalType.RETRIEVE_CARGO;
		}else if(dhdMap.Motion.GRAB_HATCH.get()){
			return GoalType.RETRIEVE_HATCH;
		}else if(dhdMap.Motion.PLACE_CARGO.get()){
			if(getGH()==GoalHeight.OVER){
				return GoalType.CARGO_CARGO;
			}else{
				return GoalType.ROCKET_CARGO;
			}
		}else if(dhdMap.Motion.PLACE_HATCH.get()){
			if(getGH()==GoalHeight.LOW){
				return GoalType.CARGO_HATCH;
			}else{
				return GoalType.ROCKET_HATCH;
			}
		}else{
			return null;
		}
	}

	public static GoalHeight getGH(){
		if(dhdMap.Motion.MID.get()){
			return GoalHeight.MIDDLE;
		}else if(dhdMap.Motion.HIGH.get()){
			return GoalHeight.HIGH;
		}else if(dhdMap.Motion.OVER.get()){
			return GoalHeight.OVER;
		}else{
			return GoalHeight.LOW;
		}
	}
}
