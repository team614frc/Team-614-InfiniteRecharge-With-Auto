package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import frc.robot.subsystems.vision.Vision;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.playingwithfusion.TimeOfFlight;

public class Shooter extends Subsystem {
  public CANSparkMax shooterMotor;
  public TimeOfFlight tofSensorShooter;

  public CANSparkMax acceleratorMotor;
  private CANPIDController shooterPIDController;
  private CANEncoder encoder;

  public Shooter() {
    shooterMotor = new CANSparkMax(RobotMap.shooterMotorPort, MotorType.kBrushless);
    acceleratorMotor = new CANSparkMax(RobotMap.acceleratorMotorPort, MotorType.kBrushless);

    tofSensorShooter = new TimeOfFlight(1);
    shooterPIDController = shooterMotor.getPIDController();
    encoder = shooterMotor.getEncoder();

    shooterPIDController.setP(RobotMap.shooterPValue);
    shooterPIDController.setI(RobotMap.shooterIValue);
    shooterPIDController.setD(RobotMap.shooterDValue);
    shooterPIDController.setIZone(RobotMap.shooterIZValue);
    shooterPIDController.setFF(RobotMap.shooterFFValue);
    shooterPIDController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput);
  }

  public void initDefaultCommand() {
    // setDefaultCommand(new DeliverGoalHigh());
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void setAcceleratorSpeed(double speed) {
    acceleratorMotor.set(speed);
  }

  public void createTelemetry() {
    SmartDashboard.putNumber("Shooter: P Value", RobotMap.shooterPValue);
    SmartDashboard.putNumber("Shooter: I Value", RobotMap.shooterIValue);
    SmartDashboard.putNumber("Shooter: D Value", RobotMap.shooterDValue);
    SmartDashboard.putNumber("Shooter: FF Value", RobotMap.shooterFFValue);
    SmartDashboard.putNumber("Shooter: IZ Value", RobotMap.shooterIZValue);
  }

  public void printTelemetry() {
    SmartDashboard.putNumber("Shooter: P Value Telemetry", shooterPIDController.getP());
    SmartDashboard.putNumber("Shooter: I Value Telemetry", shooterPIDController.getI());
    SmartDashboard.putNumber("Shooter: D Value Telemetry", shooterPIDController.getD());
    SmartDashboard.putNumber("Shooter: FF Values Telemetry", shooterPIDController.getFF());
    SmartDashboard.putNumber("Shooter: IZ Value Telemetry", shooterPIDController.getIZone());
  }

  public void updateTelemetry() {
      shooterPIDController.setP(SmartDashboard.getNumber("Shooter: P Value", RobotMap.shooterPValue));
      shooterPIDController.setI(SmartDashboard.getNumber("Shooter: I Value", RobotMap.shooterIValue));
      shooterPIDController.setD(SmartDashboard.getNumber("Shooter: D Value", RobotMap.shooterDValue));
      shooterPIDController.setFF(SmartDashboard.getNumber("Shooter: FF Value", RobotMap.shooterFFValue));
      shooterPIDController.setIZone(SmartDashboard.getNumber("Shooter: IZ Value Set", RobotMap.shooterIZValue));
  }

  public void setShooterReference(double setPoint) {
    shooterPIDController.setReference(setPoint, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter: Process Variable", encoder.getVelocity());
    SmartDashboard.putNumber("Shooter: Setpoint of Current Shot", setPoint);
  }

  public void followShooter() {
    acceleratorMotor.follow(shooterMotor);
  }

  // public void setIntakeSpeed(double speed) {
  // intakeMotor.set(speed);
  // }

}