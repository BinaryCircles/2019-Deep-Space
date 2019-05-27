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

  public WPI_TalonSRX talon_fl;
  public WPI_VictorSPX victor_bl;

  public WPI_TalonSRX talon_fr;
  public WPI_VictorSPX victor_br;

  DifferentialDrive d_drive;
  public AHRS ahrs;
  public boolean inverted;

  public DriveSubsystem()
  {
    super("Drive Subsystem");
    talon_fl = new WPI_TalonSRX(RobotMap.talon_fl);
    victor_bl = new WPI_VictorSPX(RobotMap.victor_bl);
    talon_fr = new WPI_TalonSRX(RobotMap.talon_fr);
    victor_br = new WPI_VictorSPX(RobotMap.victor_br);
    d_drive = new DifferentialDrive(talon_fr, talon_fl);
    inverted = false;
    new AHRS(SPI.Port.kMXP);
    victor_bl.follow(talon_fl);
    victor_br.follow(talon_fr);
    talon_fl.setInverted(inverted);
    talon_fr.setInverted(inverted);
    victor_bl.setInverted(inverted);
    victor_br.setInverted(inverted);
    talon_fl.configPeakCurrentLimit(80);
    talon_fr.configPeakCurrentLimit(80);
    talon_fl.configPeakCurrentDuration(50);
    talon_fr.configPeakCurrentDuration(50);
    talon_fl.enableCurrentLimit(true);
    talon_fr.enableCurrentLimit(true);

    talon_fl.setSafetyEnabled(false);
    talon_fr.setSafetyEnabled(false);
    victor_bl.setSafetyEnabled(false);
    victor_br.setSafetyEnabled(false);
  }

  public boolean boostEnabled = false;

  public double sineM = 0;
  
  //drive = new RobotDrive(talon_fl, talon_bl, talon_fr, talon_br);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveCommand());
   
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed) {
    d_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    if (boostEnabled) {
      d_drive.curvatureDrive(xSpeed, zRotation * 0.8, isQuickTurn);
    } else {
      d_drive.curvatureDrive(xSpeed * 0.8, zRotation * 0.8, isQuickTurn);
    }
  }

  public void enableBoost(boolean bPressed) {
    if (bPressed) {
      boostEnabled = true;
    } else {
      boostEnabled = false;
    }
  }

  public void invertDirection() {
    inverted = !inverted;
    talon_fl.setInverted(inverted);
    talon_fr.setInverted(inverted);
    victor_bl.setInverted(inverted);
    victor_br.setInverted(inverted);
  }
 
}
