// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.Set;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final CANSparkMax m_leadMotorleft = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_shoulder_lead = new CANSparkMax(6, MotorType.kBrushed);
  private final CANSparkMax m_shoulder_follow = new CANSparkMax(7, MotorType.kBrushed);
  private final CANSparkMax m_followMotorleft = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_leadMotorright = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_followMotorright = new CANSparkMax(4, MotorType.kBrushed);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leadMotorleft, m_leadMotorright);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
 //private MotorControllerGroup shoulders = new MotorControllerGroup(m_shoulder_lead, m_shoulder_follow );
 private final Pose2d m_autoStartPose = new Pose2d();
 private final double m_autoDriveTime_sec = 2.0;
 private final double m_autoDriveSpeed_mps = 1.0;
 
 //pneumatics
  private final Compressor comp = new Compressor(6,PneumaticsModuleType.REVPH);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(6,PneumaticsModuleType.REVPH, 0, 1); //first parameter is teh CAN id

 Thread m_visionThread;

 /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_leadMotorleft.restoreFactoryDefaults();
    // m_leadMotorright.restoreFactoryDefaults();

  comp.enableDigital();
   
  boolean pressureSwitch = comp.getPressureSwitchValue();
  double current = comp.getCurrent();
  SmartDashboard.putNumber("Compresser Current", current);
  SmartDashboard.putBoolean("preasureSwitch", pressureSwitch);

  //Syncs the left side of the motors with one another
  m_followMotorleft.follow(m_leadMotorleft);

  // Syncs the right side of the motors with one another
  m_followMotorright.follow(m_leadMotorright);

  m_shoulder_follow.follow(m_shoulder_lead);

// Code for camera to recogize/read apriltags 
  m_visionThread =
  new Thread(
      () -> {
        var camera = CameraServer.startAutomaticCapture();

        var cameraWidth = 640;
        var cameraHeight = 480;

        camera.setResolution(cameraWidth, cameraHeight);

        var cvSink = CameraServer.getVideo();
        var outputStream = CameraServer.putVideo("RioApriltags", cameraWidth, cameraHeight);

        var mat = new Mat();
        var grayMat = new Mat();

        var pt0 = new Point();
        var pt1 = new Point();
        var pt2 = new Point();
        var pt3 = new Point();
        var center = new Point();
        var red = new Scalar(0, 0, 255);
        var green = new Scalar(0, 255, 0);

        var aprilTagDetector = new AprilTagDetector();

        var config = aprilTagDetector.getConfig();
        config.quadSigma = 0.8f;
        aprilTagDetector.setConfig(config);

        var quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
        quadThreshParams.minClusterPixels = 250;
        quadThreshParams.criticalAngle *= 5; // default is 10
        quadThreshParams.maxLineFitMSE *= 1.5;
        aprilTagDetector.setQuadThresholdParameters(quadThreshParams);

        aprilTagDetector.addFamily("tag16h5");

        var timer = new Timer();
        timer.start();
        var count = 0;

        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }

          Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

          var results = aprilTagDetector.detect(grayMat);

          var set = new HashSet<>();

          for (var result: results) {
            count += 1;
            pt0.x = result.getCornerX(0);
            pt1.x = result.getCornerX(1);
            pt2.x = result.getCornerX(2);
            pt3.x = result.getCornerX(3);

            pt0.y = result.getCornerY(0);
            pt1.y = result.getCornerY(1);
            pt2.y = result.getCornerY(2);
            pt3.y = result.getCornerY(3);

            center.x = result.getCenterX();
            center.y = result.getCenterY();

            set.add(result.getId());

            Imgproc.line(mat, pt0, pt1, red, 5);
            Imgproc.line(mat, pt1, pt2, red, 5);
            Imgproc.line(mat, pt2, pt3, red, 5);
            Imgproc.line(mat, pt3, pt0, red, 5);

            Imgproc.circle(mat, center, 4, green);
            Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7);

          };

          for (var id : set){
            System.out.println("Tag: " + String.valueOf(id));
          }

          if (timer.advanceIfElapsed(1.0)){
            System.out.println("detections per second: " + String.valueOf(count));
            count = 0;
          }

          outputStream.putFrame(mat);
        }
        aprilTagDetector.close();
      });
m_visionThread.setDaemon(true);
m_visionThread.start();

  }
 // This function is run once each time the robot enters autonomous mode. 
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    System.out.println("Starting autonomous");
  }

  // This function is called periodically during autonomous. 
  @Override
  public void autonomousPeriodic() {
    // Drive for 3 seconds
    if (m_timer.get() < 3.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      //m_robotDrive.arcadeDrive(.5, 0.0, false);
      m_leadMotorleft.set(-.25);
      m_leadMotorright.set(.25);
    } 
    
     else if (m_timer.get() < 3.5 && m_timer.get() > 3.0) { 
      m_leadMotorleft.set(.5);
      m_leadMotorright.set(.5);
     }
     else if (m_timer.get() < 5.5 && m_timer.get() > 3.5)
     {m_leadMotorleft.set(-.25);
      m_leadMotorright.set(.25);}
    else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }
  
  

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    System.out.println("Starting teleop");
    //m_leadMotorleft.set(.5);
   // m_leadMotorright.set(.5);
   
  }

  /** This function is called periodically during teleoperated mode. */
  //Display right joystick value
  @Override
  public void teleopPeriodic() {
   SmartDashboard.putNumber("right joystick", m_controller.getRightY());
   //SmartDashboard.putNumber("PWM", m_shoulder_lead.get());
   // m_shoulder_lead.set (m_controller.getRightY());
   // shoulders.set (m_controller.getRightY());
    // Uses the x postion (x-axis) to go left and right and uses the y positiion (y-axis) to go up and down.
 m_robotDrive.arcadeDrive(-m_controller.getLeftX()*.7, -m_controller.getLeftY()*.7);

m_shoulder_lead.set(m_controller.getRightY());

 if (m_controller.getRightBumperPressed()){

 solenoid.set(DoubleSolenoid.Value.kForward);
 }
 else if (m_controller.getLeftBumperPressed()) {
  solenoid.set(DoubleSolenoid.Value.kReverse);}
  }
  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftX(), -m_controller.getLeftY());

  }
} 