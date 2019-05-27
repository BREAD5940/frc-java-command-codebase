///*----------------------------------------------------------------------------*/
///* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
///* Open Source Software - may be modified and shared by FRC teams. The code   */
///* must be accompanied by the FIRST BSD license file in the root directory of */
///* the project.                                                               */
///*----------------------------------------------------------------------------*/
//
//package frc.robot.subsystems;
//
//import edu.wpi.first.wpilibj.SPI;
//import org.team5940.pantry.exparimental.command.SendableSubsystemBase;
//
///**
// * Add your docs here.
// */
//public class Sensors extends SendableSubsystemBase {
//	SPI kWristEncoder, kElbowEncoder;
//	final int encoderCPR = 2048;
//
//	public Sensors(SPI.Port wristPort, SPI.Port elbowPort) {
//		this.kWristEncoder = new SPI(wristPort);
//		this.kElbowEncoder = new SPI(elbowPort);
//	}
//
//	@Override
//	public void initDefaultCommand() {
//		// Set the default command for a subsystem here.
//		// setDefaultCommand(new MySpecialCommand());
//	}
//}
