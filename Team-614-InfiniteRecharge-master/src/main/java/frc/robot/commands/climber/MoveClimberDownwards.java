package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax.IdleMode;

public class MoveClimberDownwards extends Command {

  public MoveClimberDownwards() {

  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_climber.climberMotor.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    Robot.m_climber.motorBrake.set(DoubleSolenoid.Value.kReverse);
    if (Robot.m_climber.climberMotor.getEncoder().getPosition() < 0) {
      Robot.m_climber.climberMotor.set(0.5);
    } else {
      Robot.m_climber.climberMotor.set(0);
    }
    Robot.m_climber.climberMotor.setIdleMode(IdleMode.kCoast);
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  public void end() {
    Robot.m_climber.climberMotor.set(0);
    Robot.m_climber.climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void interrupted() {
    Robot.m_climber.climberMotor.set(0);
    Robot.m_climber.climberMotor.setIdleMode(IdleMode.kBrake);
  }

}