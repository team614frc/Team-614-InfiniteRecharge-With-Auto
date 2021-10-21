package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 *
 */
public class RunClimberDown extends Command {
	public RunClimberDown() {
        requires(Robot.m_climber);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_climber.climberMotor.set(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        Robot.m_climber.runTMP(0);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.m_climber.checkLimitOne() || Robot.m_climber.checkLimitTwo();
	}
    
	// Called once after isFinished returns true
	protected void end() {
        Robot.m_climber.climberMotor.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.m_climber.climberMotor.set(0);
    }
    
    // protected void pistonToggle() {
    //     Robot.m_intake.toggleDoubleSolenoidA();
    //     Robot.m_intake.toggleDoubleSolenoidB();
    // }
}