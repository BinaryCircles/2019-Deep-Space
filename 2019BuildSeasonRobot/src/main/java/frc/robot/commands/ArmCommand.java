/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * An example command.  You can replace me with your own command.
 */
public class ArmCommand extends Command {
  public ArmCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_armsubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_armsubsystem.setCruiseAndAcceleration(10, 5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_armsubsystem.turnArm();
    if(Robot.m_oi.contr.getBumperPressed(GenericHID.Hand.kRight)) {
      Robot.m_armsubsystem.armUp(20);
    } else if (Robot.m_oi.contr.getBumperPressed(GenericHID.Hand.kLeft)) {
      Robot.m_armsubsystem.armReset();
    }
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
