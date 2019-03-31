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
import frc.robot.commands.ArmCommand;
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
  private TalonSRX armR; 
  private VictorSPX armL;
  private Encoder currentPos;
  private double kP;
  private double kI;
  private double kD;
  private double m_setpoint;
  private double startingEncoderPosition;
  private boolean rawTurnEnabled;
  private double pwr;
  //Linearizing feedforward constant
  private double kF_lin;
  //Default constructor
  public ArmSubsystemRio()
  {
    super("Arm Subsystem");
    rawTurnEnabled = false;
    kF_lin = 0.097;
    kP = 0.055;
    kI = 0.0;
    kD = 0.0;
    controller = new SynchronousPIDF(kP, kI, kD);
    armR  = new TalonSRX(RobotMap.arm_talon);
    armL = new VictorSPX(RobotMap.arm_victor);
    pwr=0;
    armR.enableCurrentLimit(true);
    armR.configPeakCurrentLimit(20);
    armR.configPeakCurrentDuration(50);
    armR = new TalonSRX(RobotMap.arm_talon);
    armL = new VictorSPX(RobotMap.arm_victor);
    armR.setInverted(true);
    armL.follow(armR);
    armL.setInverted(true);
    currentPos = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    currentPos.reset(); 
    startingEncoderPosition = -0.5;
    m_setpoint = 45;
    controller.setSetpoint(m_setpoint);
    armR.setSelectedSensorPosition(0);
    }
  public static ArmSubsystemRio getInstance()
  {
    return armSub;
  }
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArmCommand());
  }
  public void resetEncoder()
  {
    currentPos.reset();
  }

  public void setArmPos(int setpoint) {
 
      armR.configPeakCurrentLimit(1);
      m_setpoint = setpoint;
      controller.setSetpoint((double) m_setpoint );  
    
  }   
    // imagine hackeman//
  public void rawTurnArm(double power) {
    if (rawTurnEnabled) {
      armR.configPeakCurrentLimit(40);
      armR.set(ControlMode.PercentOutput, power/2, DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees())));
      pwr = (power/2);
    }
  }
public void changeRawTurnStatus() {
  rawTurnEnabled = !rawTurnEnabled;
}

  @Override
  public void periodic()
  {
    double pidOutput = controller.calculate(getPositionDegrees(), 0.02);
    // System.out.println(pidOutput);

    //armR.set(ControlMode.PercentOutput, pidOutput, DemandType.ArbitraryFeedForward, kF_lin* Math.cos(Math.toRadians(getPositionDegrees())));
    if (rawTurnEnabled) {
      armR.set(ControlMode.PercentOutput, pwr, DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees())));
    } else {
      armR.set(ControlMode.PercentOutput, pidOutput, DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees())));
      //armR.set(ControlMode.PercentOutput,  kF_lin);      
    }

    SmartDashboard.putNumber("Encoder Raw Output", getEncoderValue());
    SmartDashboard.putNumber("Arm Output", getPositionDegrees());
    SmartDashboard.putNumber("Arm pid Output", pidOutput );
    SmartDashboard.putNumber("Arm setpoint", m_setpoint );
    SmartDashboard.putNumber("PID Error", controller.getError());
    SmartDashboard.putNumber("PID Error 2", controller.getError());
  }

  public double getEncoderValue() {
    return currentPos.get();
    //return armR.getSelectedSensorPosition();
  }

  public double getPositionDegrees() {
    return (((double)currentPos.get() )* 360.0 / (4.0 * 256.0)) + startingEncoderPosition;
    //return (((double)armR.getSelectedSensorPosition())* 360.0 / (4.0 *  1024.0)) + startingEncoderPosition;
  }
}