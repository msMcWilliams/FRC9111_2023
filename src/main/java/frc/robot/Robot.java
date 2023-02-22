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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final CANSparkMax m_leadMotorleft = new CANSparkMax(1, MotorType.kBrushed);
   private final PWMSparkMax m_shoulder_lead = new PWMSparkMax(8);
  private final PWMSparkMax m_shoulder_follow = new PWMSparkMax(9);
  private final CANSparkMax m_followMotorleft = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_leadMotorright = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_followMotorright = new CANSparkMax(4, MotorType.kBrushed);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leadMotorleft, m_leadMotorright);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
 private MotorControllerGroup shoulders = new MotorControllerGroup(m_shoulder_lead, m_shoulder_follow );
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

  m_followMotorleft.follow(m_leadMotorleft);
  m_followMotorright.follow(m_leadMotorright);

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
    // Drive for 2 seconds
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
  @Override
  public void teleopPeriodic() {
SmartDashboard.putNumber("right joystick", m_controller.getRightY());
SmartDashboard.putNumber("PWM", m_shoulder_lead.get());
    m_shoulder_lead.set (m_controller.getRightY());
    shoulders.set (m_controller.getRightY());
    
 m_robotDrive.arcadeDrive(-m_controller.getLeftX()*.7, -m_controller.getLeftY()*.7);
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