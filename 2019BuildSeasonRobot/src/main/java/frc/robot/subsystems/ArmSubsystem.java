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
import frc.robot.commands.ArmCommand;

import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
* Arm subsystem controlled by Talon motor controllers.
*/

public class ArmSubsystem extends Subsystem {
  
  // initializing variables
  private SynchronousPIDF controller;
  private static TalonSRX armTalon;
  private static VictorSPX armVictor;
  private Encoder currentPos;

  // pid constants
  private static double kP, kI, kD;
  private static double kF_lin, kF_value;
  private static double m_setpoint, pidOutput, startingEncoderPosition, offset;
  
  private boolean rawTurnEnabled;

  // constructor
  public ArmSubsystem() {

    // instantiate motor controllers
    armTalon = new TalonSRX(RobotMap.arm_talon);
    armVictor = new VictorSPX(RobotMap.arm_victor);
    currentPos = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    currentPos.reset();
    resetEncoder();
    rawTurnEnabled = false;

    // initialize controller/constants
    kP = 0.018;
    kI = 0.007;
    kD = 0.0006;
    controller = new SynchronousPIDF(kP, kI, kD);
    kF_lin = 0.15; // 0.2
    kF_value = 0.15;
    startingEncoderPosition = -2;
    offset = -27;

    // configure motor controllers
    //armTalon.configFactoryDefault();
    //armVictor.configFactoryDefault();
    armTalon.configNeutralDeadband(0.0001);
    armTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armTalon.setInverted(true);
    armVictor.setInverted(false);
    armTalon.setSensorPhase(true);
    armTalon.enableCurrentLimit(true);
    armTalon.configPeakCurrentLimit(20);
    armTalon.configPeakCurrentDuration(50);
    armTalon.configContinuousCurrentLimit(20);
    
    armVictor.follow(armTalon);

    armTalon.config_kP(0, kP);
    armTalon.config_kI(0, kI);
    armTalon.config_kD(0, kD);
    
    m_setpoint = 130; // 507 encoder ticks = 45 degree
    controller.setSetpoint(m_setpoint); // temporary

  }

  // reset encoder
  public void resetEncoder() {
    armTalon.setSelectedSensorPosition(0); // -22 encoder ticks = ~-2 degrees
    currentPos.reset(); // rio pid
  }

  // reset integrator
  public void resetIntegrator() {
    controller.resetIntegrator();
  }

  // toggle feedforward
  public void toggleFeedforward() {
    if (kF_lin == kF_value) {
      kF_lin = 0;
    } else {
      kF_lin = kF_value;
    }
  }

  // toggle raw turn on/off
  public void changeRawTurnStatus() {
    rawTurnEnabled = !rawTurnEnabled;
  }

  // update variable setpoint
  public void updateSetpoint(double setpoint) {
    m_setpoint = setpoint;
    controller.setSetpoint(m_setpoint); // synchronous pid setpoint
  }

  // set pid setpoint
  public void updatePower() {
    if (!rawTurnEnabled) {
      //armTalon.set(ControlMode.Position, m_setpoint, DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees()))); // talon pid
      //armTalon.set(ControlMode.PercentOutput, kF_lin); // stall tuning script
      pidOutput = controller.calculate(getPositionDegrees(), 0.02); // synchronous pid calculate output
      armTalon.set(ControlMode.PercentOutput, pidOutput, DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees() + offset)));
    }
  }

  // move arm according to a user-set power
  public void rawTurnArm(double power) {
    if (rawTurnEnabled) {
      armTalon.set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, kF_lin * Math.cos(Math.toRadians(getPositionDegrees() + offset)));
    }
  }

  // called every 20 ms
  public void periodic() {
    updatePower();
    outputToSmartDashboard();
  }
  
  // output debugging information to shuffleboard
  public void outputToSmartDashboard() {
    //SmartDashboard.putNumber("arm loop error", armTalon.getClosedLoopError());
    //SmartDashboard.putNumber("arm encoder value", armTalon.getSelectedSensorPosition()); // talon input
    SmartDashboard.putNumber("arm loop error", controller.getError());
    SmartDashboard.putNumber("arm loop output", controller.get());
    SmartDashboard.putNumber("arm encoder value", currentPos.get()); // wire to rio
    SmartDashboard.putNumber("arm encoder calculated degrees", getPositionDegrees());
    SmartDashboard.putNumber("arm current", armTalon.getOutputCurrent());
    SmartDashboard.putNumber("arm voltage", armTalon.getMotorOutputVoltage());
    SmartDashboard.putNumber("arbitrary feedforward", kF_lin * Math.cos(Math.toRadians(getPositionDegrees() + offset)));
    SmartDashboard.putNumber("arm integral", controller.getIntegral());
  }

  // convert encoder ticks to degrees
  public double getPositionDegrees() {
    return (currentPos.get() * 360.0 / (4.0 * 256.0)) + startingEncoderPosition; // rio input
    //return (armTalon.getSelectedSensorPosition() * 360.0 / (4.0 * 1024.0)); // talon input
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
