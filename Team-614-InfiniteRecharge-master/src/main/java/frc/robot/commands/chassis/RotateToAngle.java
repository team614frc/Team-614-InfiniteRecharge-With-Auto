package frc.robot.commands.chassis;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class RotateToAngle extends Command {
  private AHRS navx;
  private PIDController pid;
  private double angle;
  private Timer timer;

  private double limelightAngle;

  public RotateToAngle(AHRS navx, double angle) {
    this.navx = navx;
    this.angle = angle;
    timer = new Timer();
    timer.start();

    pid = new PIDController(RobotMap.pValue, RobotMap.iValue, RobotMap.dValue);
    pid.setTolerance(0.01f);
    requires(Robot.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    pid.reset();
    angle = Robot.m_limelight.getX();
    // Robot.m_limelight.setLED(1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    SmartDashboard.putNumber("Value", pid.calculate(Robot.m_limelight.getX(), 0));
    // double val = pid.calculate(navx.getYaw(), angle);
    Robot.m_drivetrain.arcadeDrive(0.0, -pid.calculate(Robot.m_limelight.getX(), 0));

  }

    // Returns true when the command should end.
   public boolean isFinished() {     
    return false;
   }

  // Called once the command ends or is interrupted.
  public void end() {  
      Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
      // Robot.m_limelight.setLED(0);
  }

  public void interrupted() {
     Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
    //  Robot.m_limelight.setLED(0);
  } 
}