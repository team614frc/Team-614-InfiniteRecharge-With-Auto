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
import frc.robot.subsystems.chassis.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.serializer.*;
import frc.robot.subsystems.pneumatics.*;

import com.kauailabs.navx.frc.AHRS; 

import com.revrobotics.CANSparkMax;
import frc.robot.commands.autonomous.LeftPath;
import frc.robot.commands.shooter.ShootForATime;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.compressorcontrol.CompressorControl;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static AHRS m_navX;
  public static Pneumatics pneumatics;

  public static Shooter m_shooter;
  public static Drivetrain m_drivetrain;
  
  public static Vision m_limelight;
  public static Feeder m_feeder;
  public static OI m_oi;

  public static Intake m_intake;
  public static Climber m_climber = new Climber();
  public static Serializer m_serializer = new Serializer();

  
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
   
    try {
			m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);
		} catch (RuntimeException e) {
			DriverStation.reportError("NAVX ERROR: " + e.getMessage(), true);
    }

        
    pneumatics = new Pneumatics();
    
    m_shooter = new Shooter();
    m_drivetrain = new Drivetrain();
    m_serializer = new Serializer();
    m_limelight = new Vision();

    m_intake = new Intake();
    m_feeder = new Feeder();
    m_oi = new OI();
   
    
    
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    SmartDashboard.putNumber("Heading ", 0);
    SmartDashboard.putNumber("Number", 0);
    SmartDashboard.putNumber("Angle ", 0);
    m_drivetrain.resetDrivetrain();


  //   // Reuse buffer
  //   // Default to a length of 60, start empty output
  //   // Length is expensive to set, so only set it once, then just update data
  //   m_ledBuffer = new AddressableLEDBuffer(60);
  //   m_led.setLength(60);

  //   // Set the data
  //   m_led.setData(m_ledBuffer);
  //   for (int i = 0; i < 60; i++) {
  //     // Sets the specified LED to the RGB values for red
  //     m_ledBuffer.setRGB(i, 255, 165, 0);
  //  }
   
  //   m_led.start();
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
    // m_led.start();
    // Robot.m_limelight.setLED(3);
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
    SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
    SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
    SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    
    SmartDashboard.putNumber("Output (Left Wheels)", 0);
    SmartDashboard.putNumber("Output (Right Wheels)", 0);
    SmartDashboard.putNumber("Heading ", 0);
    SmartDashboard.putNumber("Angle ", Robot.m_navX.getAngle());

  
    // Robot.m_limelight.setLED(0);
    
    m_autonomousCommand = new LeftPath();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // SmartDashboard.putNumber("Timer", ShootForATime.timer.get());
		// SmartDashboard.putBoolean("Is Done", ShootForATime.timer.get() > 3);
    Scheduler.getInstance().run();
  }
  
  @Override

  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out
    Robot.m_limelight.setPipeline(3);
    Robot.m_drivetrain.leftMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.m_drivetrain.leftMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.m_drivetrain.rightMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
    Robot.m_drivetrain.rightMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);

    SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
    SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
    SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    
    SmartDashboard.putNumber("Output (Left Wheels)", 0);
    SmartDashboard.putNumber("Output (Right Wheels)", 0);
    SmartDashboard.putNumber("Heading ", 0);
    SmartDashboard.putNumber("Angle ", Robot.m_navX.getAngle());
    SmartDashboard.putNumber("Serializer Speed", 0);
    m_shooter.createTelemetry();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    SmartDashboard.putNumber("Angle ", Robot.m_navX.getAngle());
    m_limelight.printTelemetry();
    SmartDashboard.putNumber("Feeder Range", m_feeder.tofSensorFeeder.getRange());
    SmartDashboard.putNumber("Shooter Velocity", m_shooter.shooterMotor.getEncoder().getVelocity());
    m_feeder.printTOFSensorReading();
    SmartDashboard.putNumber("Counter", m_feeder.counter);
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("Shoooter Output", m_shooter.shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shoooter Voltage", m_shooter.shooterMotor.getBusVoltage());
    SmartDashboard.putNumber("Shoooter Applied Output", m_shooter.shooterMotor.getAppliedOutput());
    m_shooter.updateTelemetry();
    m_shooter.printTelemetry();
  }
  
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void printAutonomousTelemetry() {
    
  }
}
