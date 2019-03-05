/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.commands.IntakeCommand;

public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_VictorSPX intake_t = new WPI_VictorSPX(RobotMap.intake_t); // top row
  public WPI_VictorSPX intake_b = new WPI_VictorSPX(RobotMap.intake_b); // bottom row
  public void succ() {
    intake_t.set(0.7);
    intake_b.set(-0.7);
  }

  public void spit() {
    intake_t.set(-0.7);
    intake_b.set(0.7);
  }

  public void atRest() {
    intake_t.set(0.0);
    intake_b.set(0.0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
