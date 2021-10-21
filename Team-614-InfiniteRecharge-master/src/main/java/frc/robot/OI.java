/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//poopy butt face
package frc.robot;
import com.fasterxml.jackson.databind.deser.impl.SetterlessProperty;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.autonomous.LeftPath;
import frc.robot.commands.chassis.DriveForADistance;
import frc.robot.commands.chassis.FollowPath;
import frc.robot.commands.chassis.LimelightTurnToAngle;
import frc.robot.commands.shooter.DeliverGoalHigh;
import frc.robot.commands.shooter.DeliverGoalLow;
import frc.robot.commands.shooter.ShootForATime;
import frc.robot.commands.misc.BlinkForATime;
import frc.robot.commands.shooter.SetSpeed;
import frc.robot.commands.chassis.RotateToAngle;
import frc.robot.commands.chassis.RotateToAngleBackwards;
import frc.robot.commands.chassis.ShootSequence;
import frc.robot.commands.shooter.ShootSequence2;
import frc.robot.commands.intake.RunIntakeBasic;
import frc.robot.commands.intake.SetPistonIn;
import frc.robot.commands.intake.SetPistonOut;
import frc.robot.commands.limelight.*;
import frc.robot.commands.intake.IntakeToggle;
import frc.robot.commands.intake.IntakeSetValue;
import frc.robot.commands.intake.IntakeOnSmartDashboard;
import frc.robot.commands.serializer.RunSerializerA;
import frc.robot.commands.serializer.RunSerializerB;
import frc.robot.commands.serializer.RunSerializerAdvanced;
import frc.robot.commands.serializer.RunSerializerBasic;
import frc.robot.commands.serializer.SerializerAutomated;
import frc.robot.commands.serializer.SerializerOnSmartDashboard;
import frc.robot.commands.climber.BrakePistonIn;
import frc.robot.commands.climber.BrakePistonOut;
import frc.robot.commands.climber.MoveClimberDownwards;
import frc.robot.commands.feeder.*;
import frc.robot.commands.climber.MoveClimberUpwards;
//import frc.robot.commands.intake.runOuttake;
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
  public static final int AButton = 1;

	public static final int BButton = 2;

	public static final int XButton = 3;

	public static final int YButton = 4;

	public static final int LeftBumper = 5;

	public static final int RightBumper = 6;

	public static final int BackButton = 7;

	public static final int StartButton = 8;

  public static final int LeftStick = 9;
  
	public static final int RightStick = 10;



  //Driver Controller 
  public static final XboxController driverController = new XboxController(0);

  public static final Button RunShooterHigh = new JoystickButton(driverController, RobotMap.RightBumper);
  public static final Button RunShooterLow = new JoystickButton(driverController, RobotMap.LeftBumper);
  // public static final Button RunIntakeSerializer = new JoystickButton(driverController, RobotMap.LeftBumper);
  // public static final Button RunFeeder = new JoystickButton(driverController, RobotMap.RightBumper);
  public static final Button RunLimelightHighTurnToAngle = new JoystickButton(driverController, RobotMap.YButton);
  public static final Button RunLimelightLowTurnToAngle = new JoystickButton(driverController, RobotMap.BButton);
  public static final Button ToggleIntake = new JoystickButton(driverController, RobotMap.XButton);

  //Intake controller
  public static final XboxController operatorController = new XboxController(1);
  
  public static final Button MoveClimberUpwards = new JoystickButton(operatorController, RobotMap.LeftBumper);
  public static final Button MoveClimberDownwards = new JoystickButton(operatorController, RobotMap.RightBumper); 
  public static final Button RunFeederBackwards = new JoystickButton(operatorController, RobotMap.AButton);
  public static final Button RunIntakeBackwards = new JoystickButton(operatorController, RobotMap.BButton);
  public static final Button RunSerializerBackwards = new JoystickButton(operatorController, RobotMap.YButton);

  public OI() {
    /* Driver Controller */
    RunShooterLow.whileHeld(new ShootSequence2());
    RunShooterHigh.whileHeld(new ShootSequence());
    // RunFeeder.whileHeld(new FeederSetValue(0.35));
    RunLimelightHighTurnToAngle.whileHeld(new  RotateToAngleBackwards(Robot.m_navX, 0));
    RunLimelightLowTurnToAngle.whileHeld(new RotateToAngle(Robot.m_navX, 0));
    
    ToggleIntake.whenPressed(new IntakeToggle());
    

    /* Operator Controller */
    MoveClimberUpwards.whileHeld(new MoveClimberUpwards());
    MoveClimberDownwards.whileHeld(new MoveClimberDownwards());
    RunFeederBackwards.whileHeld(new SetSpeed(0.5));
    RunIntakeBackwards.whileHeld(new IntakeOnSmartDashboard());
    RunSerializerBackwards.whileHeld(new FeederSetValue(1.0));
  }
}

