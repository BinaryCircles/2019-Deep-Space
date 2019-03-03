/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

//import com.sun.net.ssl.TrustManager;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoDriveCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_TalonSRX talon_fl = new WPI_TalonSRX(RobotMap.talon_fl);
  public WPI_VictorSPX victor_bl = new WPI_VictorSPX(RobotMap.victor_bl);
  SpeedControllerGroup d_left = new SpeedControllerGroup(talon_fl, victor_bl);

  public WPI_TalonSRX talon_fr = new WPI_TalonSRX(RobotMap.talon_fr);
  public WPI_VictorSPX victor_br = new WPI_VictorSPX(RobotMap.victor_br);
  SpeedControllerGroup d_right = new SpeedControllerGroup(talon_fr, victor_br);

  DifferentialDrive d_drive = new DifferentialDrive(d_left, d_right);
  public AHRS ahrs = new AHRS(SPI.Port.kMXP);
  public boolean inverted = true;

  public double sineM = 0;

  //drive = new RobotDrive(talon_fl, talon_bl, talon_fr, talon_br);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveCommand());
    // Inverting these speed controller groups lets the xbox joystick
    // directions match the robot's direction.
    victor_bl.follow(talon_fl);
    victor_br.follow(talon_fr);
    d_left.setInverted(inverted);
    d_right.setInverted(inverted);

    talon_fl.configPeakOutputForward(1);
    talon_fl.configPeakOutputReverse(-1);
    talon_fr.configPeakOutputForward(1);
    talon_fr.configPeakOutputReverse(-1);

    d_drive.setSafetyEnabled(false);
    // d_drive.setExpiration(0.75);

    // arbitraryMotor.follow(talon_fl);
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed) {
    d_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    d_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  public void invertDirection() {
    inverted = !inverted;
    d_left.setInverted(inverted);
    d_right.setInverted(inverted);
  }
 
}
