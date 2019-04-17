/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.livewindow.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import frc.robot.grip.GripPipeline;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import org.opencv.core.Mat;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.IterativeRobot;
import frc.robot.commands.VisionCommand;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class VisionSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VisionThread visionThread;
  private GripPipeline pipe;
  //private GripPipeline pipeline;
  private final Object imgLock = new Object();
  private double centerX = 0.0;
  public int IMG_WIDTH = 320;
  public int IMG_HEIGHT = 240;
  // public CvSource cvOutput; 
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new VisionCommand());
  }

  public VisionSubsystem() {
      
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
    //UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    VideoSink server = CameraServer.getInstance().getServer();
    // cvOutput = CameraServer.getInstance().putVideo("processed video", IMG_WIDTH, IMG_HEIGHT);
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    /*camera.setFPS(25);
    camera2.setFPS(25);
    camera2.setResolution(IMG_WIDTH, IMG_HEIGHT);*/
    //visionThread.setDaemon(true);
    //visionThread.start();
  }

  
  /*public void startVision() {
    if(Robot.m_oi.contr.getRawButtonReleased(11)){
    

    
    }

  //   UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
  //   camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
  //   camera.setFPS(25);
    
  //   visionThread = new VisionThread(camera, pipe, pipeline -> {
  //     if (!pipeline.filterContoursOutput().isEmpty()) {
  //         Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
  //         synchronized (imgLock) {
  //             centerX = r.x + (r.width / 2);
              
  //         }
  //     }
  //   });
    
  // visionThread.setDaemon(true);
  // visionThread.start();
}*/

  public void useSee() {
  
      // SmartDashboard.putString("debugging message,", "hello uwu :3");
      // CvSink cvSink = CameraServer.getInstance().getVideo();
      // // cvOutput = CameraServer.getInstance().putVideo("processed video", IMG_WIDTH, IMG_HEIGHT);
     
      // Mat mat = new Mat();
      
    	// if (cvSink.grabFrame(mat) == 0)
    	// { 
      //   SmartDashboard.putString("error", cvSink.getError());        
    	// }
    	// else
    	// {
          
      //   	GripPipeline grip = new GripPipeline();
      //   	grip.process(mat);
      //     //System.out.println(grip.filterContoursOutput().size());
      //     SmartDashboard.putString("detect", "mat detected");
      //     // cvOutput.putFrame(mat);
      //     double turn = this.centerX - (IMG_WIDTH / 2);
      //     SmartDashboard.putNumber("turn", turn);
      //     // Robot.m_drivesub.curvatureDrive(0, turn*0.5, true);
          

    	// }
  }
}
  