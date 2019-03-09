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
    d_drive = new DifferentialDrive(talon_fr,talon_fl);
    inverted = true;
    new AHRS(SPI.Port.kMXP);
    victor_bl.follow(talon_fl);
    victor_br.follow(talon_fr);
    talon_fl.setInverted(inverted);
    talon_fr.setInverted(inverted);

    talon_fl.configPeakOutputForward(1);
    talon_fr.configPeakOutputReverse(-1);
    talon_fr.configPeakOutputForward(1);
    talon_fl.configPeakOutputReverse(-1);

    d_drive.setSafetyEnabled(false);
  }

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
    d_drive.curvatureDrive(xSpeed, -1 *zRotation, isQuickTurn);
  }

  public void invertDirection() {
    inverted = !inverted;
    talon_fl.setInverted(inverted);
    talon_fr.setInverted(inverted);
  }
 
}
