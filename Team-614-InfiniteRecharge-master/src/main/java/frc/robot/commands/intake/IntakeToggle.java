package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 *
 */
public class IntakeToggle extends Command {
	public IntakeToggle() {
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// Robot.arm.sparkMaxB.setInverted(true);
	  RobotMap.isDumb = !RobotMap.isDumb;
      if(RobotMap.isDumb) {
        Robot.m_intake.intakeSolenoidA.set(DoubleSolenoid.Value.kReverse);
        Robot.m_intake.intakeSolenoidB.set(DoubleSolenoid.Value.kForward);
      } else {
        Robot.m_intake.intakeSolenoidA.set(DoubleSolenoid.Value.kForward);
        Robot.m_intake.intakeSolenoidB.set(DoubleSolenoid.Value.kReverse);
      }
	}


	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}
    
	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
    }
    
    // protected void pistonToggle() {
    //     Robot.m_intake.toggleDoubleSolenoidA();
    //     Robot.m_intake.toggleDoubleSolenoidB();
    // }
}