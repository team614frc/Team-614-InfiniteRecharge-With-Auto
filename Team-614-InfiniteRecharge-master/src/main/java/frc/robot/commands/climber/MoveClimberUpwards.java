package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.Timer;
public class MoveClimberUpwards extends Command {
   
  Timer timer; 

  public MoveClimberUpwards() {
    timer = new Timer();
    
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_climber.climberMotor.set(0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    if(Robot.m_climber.limitSwitch1.get() && Robot.m_climber.limitSwitch2.get()) {
      Robot.m_climber.motorBrake.set(DoubleSolenoid.Value.kForward);
      if(timer.get() > .2) {
      Robot.m_climber.climberMotor.setIdleMode(IdleMode.kCoast);
      Robot.m_climber.climberMotor.set(-0.7);
       }
    } else {
      Robot.m_climber.climberMotor.set(0);
    }
  }

    // Returns true when the command should end.
   public boolean isFinished() {
	  return !Robot.m_climber.limitSwitch1.get() || !Robot.m_climber.limitSwitch2.get() ;
   }

  // Called once the command ends or is interrupted.
  public void end() {  
    Robot.m_climber.climberMotor.set(0);
    Robot.m_climber.climberMotor.setIdleMode(IdleMode.kBrake);
    Robot.m_climber.motorBrake.set(DoubleSolenoid.Value.kReverse);
  }

  public void interrupted() {
    Robot.m_climber.climberMotor.set(0);
    Robot.m_climber.climberMotor.setIdleMode(IdleMode.kBrake);
    Robot.m_climber.motorBrake.set(DoubleSolenoid.Value.kReverse);
  } 


}