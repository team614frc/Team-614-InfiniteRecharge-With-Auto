package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.chassis.Drivetrain;

// 	NAVX DEGREE ORIENTATION:
// 			 0
// 	    -45 \|/ +45
// 	  -90  --X--  +90
//	   -135 /|\ +135
//		  +/-180

public class TurnToTarget extends Command {
	private int pipeline;
	private int camMode;

	public TurnToTarget(int pipeline, int camMode) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_drivetrain);
		requires(Robot.m_limelight);
		// requires(Robot.arm);

		this.pipeline = pipeline;
		this.camMode = camMode;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.m_limelight.setPipeline(pipeline);
		Robot.m_limelight.setCamMode(camMode);
		// Robot.navX.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		// Robot.drivetrain.runCollisionDetection();
		double c = Robot.m_limelight.getX() < 0 ? -0.35 : 0.35;
		double forward = Math.abs(Robot.m_limelight.getDistance() * 0.0035) + 0.35;
		Robot.m_drivetrain.arcadeDrive(0.0, -((Robot.m_limelight.getX() * 0.03)));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// return Robot.vision.getDistance() < 10;
		// return this.timeSinceInitialized() > 1.5 && (Math.abs(Robot.drivetrain.getCurrentJerkY()) > 0.6f
		// 		|| Math.abs(Robot.drivetrain.getCurrentJerkX()) > 0.6f);
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		// Robot.drivetrain.resetSpeed();
		Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
		// Robot.m_limelight.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		// Robot.drivetrain.resetSpeed();
		Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
		// Robot.m_limelight.stop();
	}
}