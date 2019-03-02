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
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ArmSubsystem extends Subsystem 
  {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX armR = new WPI_TalonSRX(RobotMap.arm_talon);
  private WPI_VictorSPX armL = new WPI_VictorSPX(RobotMap.arm_victor);

  private SpeedControllerGroup arm = new SpeedControllerGroup(armR, armL);

  private double kP = 0;
  private double kI;
  private double kD;
  private double kF;
  public int kIzone;
  public double kPeakOutput;
  public boolean armToggle = false;
  public double currentPosition;
  // private PowerDistributionPanel pdp = new PowerDistributionPanel();

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    armR.setSafetyEnabled(false);
    armL.setSafetyEnabled(false);
    armL.setInverted(false);
    armL.follow(armR);

    // armR.configVoltageCompSaturation(4);
    armR.configPeakOutputForward(1); 
    //   // armR.configVoltageCompSaturation(0);
    // // armL.configVoltageCompSaturation(4);
    armL.configPeakOutputForward(1);
    // armL.set(0.125);
     
  }
  public void setCruiseAndAcceleration(int accel, int cruise) {  
    armR.config_kP(0, kP);
    armR.config_kI(0, kI);
    armR.config_kD(0, kD);
    armR.config_kF(0, kF);
    armR.configMotionAcceleration(accel);
    armR.configMotionCruiseVelocity(cruise);
  }

  /*public void toggleArm(double turnPosition) {
    if (armToggle == true) {
      armR.set(ControlMode.MotionMagic, turnPosition);
    } else {
      armR.set(ControlMode.MotionMagic, 0);
    }

    armToggle = !armToggle;
  }*/

  public void armUp(double turnPosition) {
    armR.set(ControlMode.Position, 30, DemandType.ArbitraryFeedForward, -0.125 * Math.cos(Math.toRadians(getPosition())));
  }

  public void armReset() {
    armR.set(ControlMode.MotionMagic, 0);
  }
    // imagine hackeman//
  public void turnArm() {
    armR.set(Robot.m_oi.getYMagnitudeOfRightSide()* 0.4);
    // armR.setSafetyEnabled(true);
    // armR.setExpiration(1);
    /*SmartDashboard.putNumber("motor output voltage", armR.getMotorOutputVoltage());    
    
    SmartDashboard.putNumber("motor output voltage left", armL.getMotorOutputVoltage());    

    SmartDashboard.putNumber("motor output current", armR.getOutputCurrent());    
    SmartDashboard.putNumber("pdp current for talon", pdp.getCurrent(4)); 
    SmartDashboard.putNumber("pdp current for victor", pdp.getCurrent(5)); 
    
    
    // SmartDashboard.putNumber("motor output voltage left", armL.getOutp);   */

  }

  public double getPosition() {
    currentPosition = armR.getSelectedSensorPosition() * 360 / (4 * 600);
    return currentPosition;
  }



}
