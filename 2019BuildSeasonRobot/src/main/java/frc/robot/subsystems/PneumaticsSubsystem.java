/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// import necessary libraries
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;

// initiate pneumatics subsystem
public class PneumaticsSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DoubleSolenoid pDouble = new DoubleSolenoid(RobotMap.solenoid_front, RobotMap.solenoid_back);
  
  private boolean arm_extended_flag = false;

  public void initialize() {

    // !make sure to test which kind solenoid it is!

    pDouble.set(DoubleSolenoid.Value.kOff);
    pDouble.set(DoubleSolenoid.Value.kForward); // solenoid in
    //pDouble.set(DoubleSolenoid.Value.kReverse); // solenoid out

  }

  public void initDefaultCommand() {

  }

  public void toggleArm() {
    if (arm_extended_flag == true) {
      //might need to change this from kReverse to kForward or kOff based on solenoid type
      pDouble.set(DoubleSolenoid.Value.kReverse);
    } else {
      //might need to change this from kReverse to kForward or kOff based on solenoid type
      pDouble.set(DoubleSolenoid.Value.kForward);
    }
    arm_extended_flag = !arm_extended_flag; 
  } 

}
