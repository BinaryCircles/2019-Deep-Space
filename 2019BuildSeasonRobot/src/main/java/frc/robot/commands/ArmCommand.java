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
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if (Robot.m_oi.joystick.getBumperPressed(GenericHID.Hand.kLeft)) {
      Robot.m_armsubsystem.changeRawTurnStatus();
    }

    //Robot.m_armsubsystem.setArmPos(0);

    if (Robot.m_oi.joystick.getAButtonPressed()) { // hatch forward
      Robot.m_armsubsystem.setArmPos(-4);
    } else if (Robot.m_oi.joystick.getBButtonPressed()) { // ship cargo forward
      Robot.m_armsubsystem.setArmPos(60);
    } else if (Robot.m_oi.joystick.getXButtonPressed()) { // rocket cargo forward
      Robot.m_armsubsystem.setArmPos(30);
    } else if (Robot.m_oi.joystick.getPOV() == 0) { // starting pos
      Robot.m_armsubsystem.setArmPos(118);
    } else if (Robot.m_oi.joystick.getPOV() == 270) { // rocket cargo passthrough
      Robot.m_armsubsystem.setArmPos(150);
    } else if (Robot.m_oi.joystick.getPOV() == 90) { // ship cargo passthrough
      Robot.m_armsubsystem.setArmPos(120);
    } else if (Robot.m_oi.joystick.getPOV() == 180) { // hatch passthrough
      Robot.m_armsubsystem.setArmPos(155);
    } else if (Robot.m_oi.joystick.getYButtonPressed()) {
      Robot.m_armsubsystem.setArmPos(-10);
    }
    SmartDashboard.putNumber("arm encoder value", Robot.m_armsubsystem.getEncoderValue());
    System.out.println(Robot.m_armsubsystem.getEncoderValue());
    Robot.m_armsubsystem.rawTurnArm(-Robot.m_oi.joystick.getY(GenericHID.Hand.kLeft) * 0.4);
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
