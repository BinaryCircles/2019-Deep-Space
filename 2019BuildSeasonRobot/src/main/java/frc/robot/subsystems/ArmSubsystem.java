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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.can;    

/**
 * Arm subsystem
 */
public class ArmSubsystem extends Subsystem 
  {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX armR;
  private WPI_VictorSPX armL;
  // PID constants
  private double kP;
  private double kI;
  private double kD;
  //Linearizing feedforward constant;
  private double kF_lin;
  //Default constructor, try not to use
  public ArmSubsystem()
  {
    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kf_lin = 0.125;
    armR = new WPI_TalonSRX(RobotMap.arm_talon);
    armL = new WPI_VictorSPX(RobotMap.arm_victor);
    armR.setSafetyEnabled(false);
    armL.setSafetyEnabled(false);
    armL.setInverted(false);
    armL.follow(armR);
    armR.configPeakOutputForward(1); 
    armL.configPeakOutputForward(1);
  }
  //Constructor with P, I, D, and F values;
  public ArmSubsystem(double P; double I; double D; double F_lin; double accel; double cruise)
  {
    // Config PID + arbitrary FF constants
    kP = P;
    kI = I;
    kD = D;
    kf_lin = F_lin;
    //Init motor controllers
    armR = new WPI_TalonSRX(RobotMap.arm_talon);
    armL = new WPI_VictorSPX(RobotMap.arm_victor);
    //set motion magic and talon PIDF constants
    armR.configMotionAcceleration(accel);
    armR.configMotionCruiseVelocity(cruise);
    armR.config_kP(kP);
    armR.config_kI(kI);
    armR.config_kD(kD);
    armR.config_kF(0);
    //misc "talons please dont mess up"
    armR.setSafetyEnabled(false);
    armL.setSafetyEnabled(false);
    armL.setInverted(false);
    armL.follow(armR);
    armR.configPeakOutputForward(1); 
    armL.configPeakOutputForward(1);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
     
  }
 


  public void setArmPos(double setpoint) {
    armR.set(ControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, kf_lin * Math.cos(Math.toRadians(getPosition())));
  }

  public void armReset() {
    armR.set(ControlMode.MotionMagic, 0);
  }
    // imagine hackeman//
  public void rawTurnArm(double power) {
    armR.set(power);

  }

  public double getPositionDegrees() {
    return armR.getSelectedSensorPosition() * 360 / (4 * 600);
  }



}
