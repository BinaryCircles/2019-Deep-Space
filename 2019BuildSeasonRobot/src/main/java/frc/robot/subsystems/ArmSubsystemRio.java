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
public class ArmSubsystem extends Subsystem 
  {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static ArmSubsystem armSub = new ArmSubsystem();
  private static TalonSRX armR; 
  private static VictorSPX armL;
  // PID constants
  private static double kP;
  private static double kI;
  private static double kD;
  //Linearizing feedforward constant;
  private static double kF_lin;
  //Default constructor
  public ArmSubsystem()
  {
    super("Arm Subsystem");
    armR  = new TalonSRX(RobotMap.arm_talon);
    armL = new VictorSPX(RobotMap.arm_victor);
    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kf_lin = 0.125;
    armR = new TalonSRX(RobotMap.arm_talon);
    armL = new VictorSPX(RobotMap.arm_victor);
    armR.setSafetyEnabled(false);
    armL.setSafetyEnabled(false);
    armL.setInverted(false);
    armL.follow(armR);
    armR.configPeakOutputForward(1); 
    armL.configPeakOutputForward(1);
  }
  public static ArmSubsystem getInstance()
  {
    return armSub;
  }
  @Override
  public void initDefaultCommand() {
  }
 


  public void setArmPos(double setpoint) {
    
  }
    // imagine hackeman//
  public void rawTurnArm(double power) {
    armR.set(power);
  }
  @Override
  public void periodic()
  {}
  public double getPositionDegrees() {
    return armR.getSelectedSensorPosition() * 360 / (4* 5 * 600);
  }
}
