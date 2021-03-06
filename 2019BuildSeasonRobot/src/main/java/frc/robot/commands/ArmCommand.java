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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    //Robot.m_armsubsystem.resetEncoder();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if (Robot.m_oi.joystick.getBumperPressed(GenericHID.Hand.kLeft)) {
      Robot.m_armsubsystem.changeRawTurnStatus();
    }

    if (Robot.m_oi.joystick.getAButtonPressed()) { // hatch forward
      Robot.m_armsubsystem.updateSetpoint(2);
    } else if (Robot.m_oi.joystick.getBButtonPressed()) { // ship cargo forward
      Robot.m_armsubsystem.updateSetpoint(60);
    } else if (Robot.m_oi.joystick.getXButtonPressed()) { // rocket cargo forward
      Robot.m_armsubsystem.updateSetpoint(40);
    } else if (Robot.m_oi.joystick.getPOV() == 0) { // starting pos
      Robot.m_armsubsystem.updateSetpoint(125);
    } else if (Robot.m_oi.joystick.getPOV() == 270) { // rocket cargo passthrough
      //Robot.m_armsubsystem.updateSetpoint(150);
    } else if (Robot.m_oi.joystick.getPOV() == 90) { // ship cargo passthrough
      Robot.m_armsubsystem.updateSetpoint(130);
    } else if (Robot.m_oi.joystick.getPOV() == 180) { // hatch passthrough
      //Robot.m_armsubsystem.updateSetpoint(155);
    }

    if (Robot.m_oi.joystick.getStartButtonPressed()) {
      Robot.m_armsubsystem.resetEncoder();
    }

    if (Robot.m_oi.joystick.getBackButtonPressed()) {
      Robot.m_armsubsystem.toggleFeedforward();
    }

    Robot.m_armsubsystem.rawTurnArm(Robot.m_oi.getYMagnitudeOfJoystickLeftSide() * -1);
    //Robot.m_armsubsystem.outputToSmartDashboard();

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
