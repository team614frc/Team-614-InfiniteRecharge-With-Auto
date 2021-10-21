package frc.robot.commands.chassis;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RotateToAngle extends Command {

  private AHRS navx;
  private PIDController pid;
  private double angle;

  public RotateToAngle(AHRS navx, double angle) {
    this.navx = navx;
    this.angle = angle;

    pid = new PIDController(5e-5,1e-6, 0);
    pid.setSetpoint(angle);
    pid.setTolerance(0.1f);
    requires(Robot.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    navx.zeroYaw();
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    double val = pid.calculate(navx.getYaw(), angle);
    Robot.m_drivetrain.arcadeDrive(0.0, val);

  }

    // Returns true when the command should end.
   public boolean isFinished() {
	  return pid.atSetpoint();
   }

  // Called once the command ends or is interrupted.
  public void end() {  
	Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
  }

  public void interrupted() {
	Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
  } 


}