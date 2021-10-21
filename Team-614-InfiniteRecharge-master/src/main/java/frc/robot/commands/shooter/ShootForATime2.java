package frc.robot.commands.shooter;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class ShootForATime2 extends Command {
	Timer timer;
	Timer timer2;
	double time;

	public ShootForATime2(double time) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_shooter);
		requires(Robot.m_feeder);
		requires(Robot.m_drivetrain);
		timer = new Timer();
		timer.reset();
		timer2 = new Timer();
		timer2.reset();
		this.time = time;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		timer.start();
		timer2.start();
        // Robot.m_limelight.setLED(2);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_shooter.setShooterSpeed(RobotMap.lowGoalSpeed);
		if (timer2.get() > 1.5) {
			Robot.m_shooter.acceleratorMotor.set(.7);
			if (Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > 3000) {
				Robot.m_feeder.feederMotor.set(.6);
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return timer.get() > time;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_shooter.shooterMotor.set(0);
		Robot.m_shooter.acceleratorMotor.set(0);
		Robot.m_feeder.feederMotor.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_shooter.shooterMotor.set(0);
		Robot.m_shooter.acceleratorMotor.set(0);
		Robot.m_feeder.feederMotor.set(0);
	}
}
