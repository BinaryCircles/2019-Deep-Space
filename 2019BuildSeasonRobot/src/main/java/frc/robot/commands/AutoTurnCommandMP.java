package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PneumaticsCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.*;
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;
public class AutoTurnCommandMP extends Command implements PIDSource, PIDOutput {
    private double rampRate;
    private double cruiseVelocity;
    private double currentVelocity;
    private double rampTicks;
    private double cruiseTicks;
    private double target;
    private int stage;
    private double time;
    public PIDController turnPID = new PIDController(0.02, 0.0, 0.0, this, this, 2665);
    private AHRS ahrs = Robot.m_drivesub.ahrs;
    private double rate;
    private double velocityError;
    
    public AutoTurnCommandMP(double cruise, double ramp) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.m_drivesub);

        //if (ahrs.getAngle() >= 0) {
        //    target = ahrs.getAngle() - 180;
        //} else {
         //   target = ahrs.getAngle() + 180;
        //}

        target = 180;
     
        rampRate = ramp;
        cruiseVelocity = cruise;
        currentVelocity = 0.0;
        rampTicks = cruiseVelocity * 50/rampRate;
        cruiseTicks = 50 * ((target - ((rampTicks/50) * cruiseVelocity))/cruiseVelocity);
        time = 0.0;
        stage = 1;
    }

    public double pidGet() {
        return ahrs.getRate();
    }

    public void pidWrite(double input) {
        velocityError = input;
    }

    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }


    public void setPIDSourceType(PIDSourceType source) {
        source = PIDSourceType.kDisplacement;
    }
    // Called just before this Command runs the first time
    // @Override
    protected void initialize() {
        setPIDSourceType(PIDSourceType.kDisplacement);
        turnPID.setInputRange(-1, 1);
        turnPID.setContinuous(true);
        turnPID.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    // @Override
    protected void execute() {
        if(stage == 1)
        {
            SmartDashboard.putString("stage 1", "stage 1");
            time += 1.0;
            currentVelocity = currentVelocity + (rampRate/50);
            turnPID.setSetpoint(currentVelocity);
            if(time > rampTicks)
            {
                stage = 2;
            }
        }
        else if(stage == 2)
        {
            SmartDashboard.putString("stage 2", "stage 2");
            time += 1.0;
            turnPID.setSetpoint(currentVelocity);
            if(time >= cruiseTicks + rampTicks)
            {
                stage = 3;
            }
        }
        else if(stage == 3)
        {
            SmartDashboard.putString("stage 3", "stage 3");
            time += 1.0;
            currentVelocity = currentVelocity - (rampRate/50);
            turnPID.setSetpoint(currentVelocity);
            if(time > ((2 *rampTicks) + cruiseTicks))
            {
                stage = 4;
            }
        }
        SmartDashboard.putNumber("velError", velocityError);
        SmartDashboard.putNumber("current angle", Robot.m_drivesub.ahrs.getAngle());
        Robot.m_drivesub.curvatureDrive(0, (currentVelocity + velocityError), true);

    }

    // Make this return true when this Command no longer needs to run execute()

    protected boolean isFinished() {
        return (stage == 4);
    }

    // Called once after isFinished returns true
    // @Override
    protected void end() {
        turnPID.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    // @Override
    protected void interrupted() {
    }
}
