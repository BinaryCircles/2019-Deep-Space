/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import edu.wpi.first.hal.can.CANMessageNotAllowedException;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can;    

/**
 * Arm subsystem
 */
public class ArmSubsystemRio extends Subsystem 
  {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private SynchronousPIDF controller;
  private static ArmSubsystemRio armSub = new ArmSubsystemRio();
  private  TalonSRX armR; 
  private  VictorSPX armL;
  private Encoder currentPos;
  private  double kP;
  private  double kI;
  private  double kD;
  private int m_setpoint;
  private int startingEncoderPosition;
  //Linearizing feedforward constant
  private double kF_lin;
  //Default constructor
  public ArmSubsystemRio()
  {
    super("Arm Subsystem");
    armR  = new TalonSRX(RobotMap.arm_talon);
    armL = new VictorSPX(RobotMap.arm_victor);
    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kF_lin = 0.125;
    armR = new TalonSRX(RobotMap.arm_talon);
    armL = new VictorSPX(RobotMap.arm_victor);
    armL.setInverted(false);
    armL.follow(armR);
    armR.configPeakOutputForward(1); 
    armL.configPeakOutputForward(1);
    currentPos = new Encoder(0, 1,false, Encoder.EncodingType.k4X );
    controller = new SynchronousPIDF(kP, kI, kD);
  }
  public static ArmSubsystemRio getInstance()
  {
    return armSub;
  }
  @Override
  public void initDefaultCommand() {
  }
 

  public void setArmPos(int setpoint) {
    m_setpoint = setpoint;
    controller.setSetpoint((double) m_setpoint);
  }
    // imagine hackeman//
  public void rawTurnArm(double power) {
    armR.set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees())));
  }
  @Override
  public void periodic()
  {
    armR.set(ControlMode.PercentOutput, controller.calculate(getPositionDegrees(), 20), DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees())));
  }
  public double getPositionDegrees() {
    return (currentPos.get() + startingEncoderPosition) * 360 / (4* 5 * 600);
  }
}
