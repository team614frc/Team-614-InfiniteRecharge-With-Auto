package frc.robot.subsystems.compressorcontrol;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.pneumatics.*;


import frc.robot.Robot;

public class CompressorControl extends Command {
	public CompressorControl() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.pneumatics);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.pneumatics.compressor.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.pneumatics.compressor.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.pneumatics.compressor.stop();
	}
}