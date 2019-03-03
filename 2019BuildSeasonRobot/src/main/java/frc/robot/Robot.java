/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PneumaticsCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoTurnCommandMP;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static VisionSubsystem m_vissubsystem = new VisionSubsystem();
  public static VisionCommand m_viscomm = new VisionCommand();

  public static PneumaticsSubsystem m_pnsub = new PneumaticsSubsystem();
  public static PneumaticsCommand m_pncomm = new PneumaticsCommand();

  public static IntakeSubsystem m_intakesub = new IntakeSubsystem();
  public static IntakeCommand m_intakecomm = new IntakeCommand();
  
  public static DriveSubsystem m_drivesub = new DriveSubsystem();
  public static DriveCommand m_drivecomm = new DriveCommand();

  public static ArmSubsystem m_armsubsystem = new ArmSubsystem();
  public static ArmCommand m_armcomm = new ArmCommand();

  public static AutoDriveCommand m_autodrivecomm = new AutoDriveCommand();
  public static AutoTurnCommandMP m_autoturncomm = new AutoTurnCommandMP(1, .1);

  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    // m_chooser.setDefaultOption("Default Auto", new AutoDriveCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

   
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.start();
    // }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    //m_autoturncomm.start();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //m_autoturncomm.cancel();
    Scheduler.getInstance().run();
    m_drivecomm.start();
    //  m_viscomm.start();

    m_intakecomm.start();

    if (m_oi.contr.getYButtonPressed()) {
      m_pncomm.execute();
    }

    if (m_oi.contr.getPOV() == 0) {
      m_drivesub.invertDirection();
    }

    if (m_oi.contr.getPOV() == 180) {
      m_autoturncomm.start();
    }

    m_armcomm.start();
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
