/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.JoystickBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeCommand extends Command {
  public IntakeCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_intakesub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.m_oi.getSecondControllerTriggerMagnitude() < -0.3) {
      Robot.m_intakesub.succ();
    } else if (Robot.m_oi.getSecondControllerTriggerMagnitude() > 0.3) {
      Robot.m_intakesub.spit();
    } else {
      Robot.m_intakesub.atRest();
    }

    SmartDashboard.putNumber("analog input", Robot.m_oi.getSecondControllerTriggerMagnitude());
    //Robot.m_intakesub.runIntakeAnalog(Robot.m_oi.getSecondControllerTriggerMagnitude());

    /*if (Robot.m_oi.contr.getAButton()) {
      Robot.m_intakesub.succ();
    } else if (Robot.m_oi.contr.getBButton()) {
      Robot.m_intakesub.spit();
    } else {
      Robot.m_intakesub.atRest();
    } outreach controls*/
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
