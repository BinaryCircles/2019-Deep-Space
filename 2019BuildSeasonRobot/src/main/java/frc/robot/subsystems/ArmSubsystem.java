/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
* Arm subsystem controlled by Talon motor controllers.
*/

public class ArmSubsystem extends Subsystem {
  
  // initializing variables
  private static ArmSubsystem armSub = new ArmSubsystem();
  private static TalonSRX armTalon;
  private static VictorSPX armVictor;

  // pid constants
  private static double kP, kI, kD;
  private static double kF_lin;
  private static double m_setpoint;
  
  private double rawPower;
  private boolean rawTurnEnabled;

  // constructor
  public ArmSubsystem() {

    // instantiate motor controllers
    armTalon = new TalonSRX(RobotMap.arm_talon);
    armVictor = new VictorSPX(RobotMap.arm_victor);
    rawTurnEnabled = false;

    // initialize constants
    kP = 98.25;
    kI = 0.0;
    kD = 6;
    kF_lin = 0.1931;

    // configure motor controllers
    armTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armTalon.setInverted(true);
    armVictor.setInverted(true);
    armTalon.setSensorPhase(true);
    armTalon.enableCurrentLimit(true);
    armTalon.configPeakCurrentLimit(30);
    armTalon.configPeakCurrentDuration(50);
    armVictor.follow(armTalon);

    armTalon.config_kP(0, kP);
    armTalon.config_kI(0, kI);
    armTalon.config_kD(0, kD);
    
    m_setpoint = 254; // 507 encoder ticks = 45 degrees

  }
  public void encoderReset()
  {
    armTalon.setSelectedSensorPosition(-22); // -22 encoder ticks = ~-2 degrees
  }
  // toggle raw turn on/off
  public void changeRawTurnStatus() {
    rawTurnEnabled = !rawTurnEnabled;
  }

  // update variable setpoint
  public void updateSetpoint(double setpoint) {
    m_setpoint = setpoint;
  }

  // set pid setpoint
  public void updatePower() {
    if (!rawTurnEnabled) {
      armTalon.set(ControlMode.Position, m_setpoint, DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees())));  
      //armTalon.set(ControlMode.PercentOutput, kF_lin); // stall tuning script
    }
  }

  // move arm according to a user-set power
  public void rawTurnArm(double power) {
    if (rawTurnEnabled) {
      armTalon.set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees())));
    }
  }

  // called every 20 ms
  public void periodic() {
    updatePower();
    outputToSmartDashboard();
  }
  
  // output debugging information to shuffleboard
  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("arm loop error", armTalon.getClosedLoopError());
    SmartDashboard.putNumber("arm encoder value", armTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("arm encoder calculated degrees", getPositionDegrees());
    SmartDashboard.putNumber("arm current", armTalon.getOutputCurrent());
  }

  

  // convert encoder ticks to degrees
  public double getPositionDegrees() {
    return (armTalon.getSelectedSensorPosition() * 360.0 / (4.0 * 1024.0));
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
