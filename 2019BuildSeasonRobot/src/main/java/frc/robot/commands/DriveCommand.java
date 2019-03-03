/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveCommand extends Command {

  private double sineMovement = 0; // set to 0 to start out (temp)
  
  public DriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_drivesub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //sineMovement += Math.PI / 200; //add pi/400 every time execute is called to simulate sin curve movement (temp)
    curveExecute();
  }

  protected void tankExecute() {
    double lmag = Robot.m_oi.getTriggerMagnitude();
    double rmag = Robot.m_oi.getYMagnitudeOfRightSide();
    Robot.m_drivesub.tankDrive(lmag, rmag);
    SmartDashboard.putNumber("TankDriveDebug-l", lmag);
    SmartDashboard.putNumber("TankDriveDebug-r", rmag);
  }

  protected void curveExecute() {
    if (Robot.m_oi.getTriggerMagnitude() < 0) {
      Robot.m_drivesub.curvatureDrive(Robot.m_oi.getTriggerMagnitude(),
      Robot.m_oi.getXMagnitudeOfLeftSide() * -1,
      Robot.m_oi.contr.getXButton());
    } else {
      Robot.m_drivesub.curvatureDrive(Robot.m_oi.getTriggerMagnitude(),
      Robot.m_oi.getXMagnitudeOfLeftSide(),
      Robot.m_oi.contr.getXButton());
    }

    // Robot.m_drivesub.curvatureDrive(Math.sin(sineMovement),
    // 0.00, // set to 0 to disable pid, breaking in gearboxes
    // false);

    // Robot.m_drivesub.tankDrive(sineMovement, 0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
