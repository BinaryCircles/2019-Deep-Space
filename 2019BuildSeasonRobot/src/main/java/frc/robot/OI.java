/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.JoystickBase;
import frc.robot.RobotMap;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public static XboxController contr = new XboxController(RobotMap.driveJoystickLeft);
  // public static XboxController drive = new XboxController(RobotMap.driveJoystickRight);
  public static XboxController joystick = new XboxController(RobotMap.driveJoystickRight);
  /*public static Joystick joystick = new Joystick(0);
  public static JoystickButton aButton = new JoystickButton(contr, 1);
  public static JoystickButton bButton = new JoystickButton(contr, 2);
  public static JoystickButton xButton = new JoystickButton(contr, 3);
  public static JoystickButton yButton = new JoystickButton(contr, 4);
  public static JoystickButton aButton2 = new JoystickButton(joystick, 1);
  public static JoystickButton bButton2  = new JoystickButton(joystick, 2);
  public static JoystickButton jtrigger = new JoystickButton(joystick, 1);
  public static JoystickButton jside = new JoystickButton(joystick, 2);*/



  // Return the magnitude (from -1 to 1) of the left stick's Y component.
  //get left trigger input minus right triger input to determine throttle forward/backward
  public double getTriggerMagnitude() {
    return Math.pow((-contr.getTriggerAxis(GenericHID.Hand.kRight)+contr.getTriggerAxis(GenericHID.Hand.kLeft)), 3);
  }

  public double getSecondControllerTriggerMagnitude() {
    return Math.pow((-joystick.getTriggerAxis(GenericHID.Hand.kRight) + joystick.getTriggerAxis(GenericHID.Hand.kLeft)), 3);
  }


  // Return the magnitude (from -1 to 1) of the right stick's Y component

  // Return the magnitude (from -1 to 1) of the arm controller's right stick's Y component.
  public double getYMagnitudeOfJoystickLeftSide() {
    return Math.pow(joystick.getY(GenericHID.Hand.kLeft), 3);
  }
  public double getXMagnitudeOfLeftSide() {
    return Math.pow(contr.getX(GenericHID.Hand.kLeft), 3) * -1;
  }
  
  // public double getZRotation() {
  //   return contr.getY(GenericHID.Hand.kL)
  // }

 


  
}
