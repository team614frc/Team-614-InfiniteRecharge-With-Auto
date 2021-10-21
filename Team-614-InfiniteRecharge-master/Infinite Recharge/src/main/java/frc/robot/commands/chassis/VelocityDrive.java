// package org.usfirst.frc.team614.robot.commands.drivetrain;

// import org.usfirst.frc.team614.robot.Robot;

// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// //
// // 	NAVX DEGREE ORIENTATION:
// // 			 0
// // 	    -45 \|/ +45
// // 	  -90  --X--  +90
// //	   -135 /|\ +135
// //		  +/-180
// // X is the robot. at +/-180, the left is -179.9999... and the right is +179.9999...

// public class RotateToAngle extends Command {

// 	double angle = 0;
// 	boolean useAbsoluteAngle = false;

// 	public RotateToAngle(double angle, boolean useAbsoluteAngle) {
// 		// Use requires() here to declare subsystem dependencies
// 		// eg. requires(chassis);
// 		requires(Robot.drivetrain);
// 		this.angle = angle;
// 		this.useAbsoluteAngle = useAbsoluteAngle;
// 	}

// 	// Called just before this Command runs the first time
// 	protected void initialize() {
// //		 Robot.navX.reset();
// //		 Robot.navX.zeroYaw();

// 		Robot.drivetrain.setUsingTurnPID(true);

// 		if (useAbsoluteAngle) {
// 			Robot.drivetrain.getTurnController().setSetpoint(angle % 360);
// 		} else { // relative angle
// 			Robot.drivetrain.getTurnController().setSetpoint((Robot.navX.getYaw() + angle));// % 360);
// 		}
// 	}

// 	// Called repeatedly when this Command is scheduled to run
// 	protected void execute() {
// 		double speed = 0.5 * Robot.drivetrain.getPIDRotateRate();
		
// 		if (speed < 0.35 && speed > 0) {
// 			speed = 0.35;
// 		} else if (speed > -0.35 && speed < 0) {
// 			speed = -0.35;
// 		}
		
// 		Robot.drivetrain.arcadeDrive(0.0, speed);
// //		Robot.drivetrain.arcadeDrive(0.0, Robot.drivetrain.getPIDRotateRate());
// 	}

// 	// Make this return true when this Command no longer needs to run execute()
// 	protected boolean isFinished() {
// 		return false;
// //		return timeSinceInitialized() > 1.5 && Robot.drivetrain.leftEncoder.getRate() < 1 && Robot.drivetrain.leftEncoder.getRate() > -1;
// 	}

// 	// Called once after isFinished returns true
// 	protected void end() {
// 		Robot.drivetrain.setUsingTurnPID(false);
// 		Robot.drivetrain.stop();
// 	}

// 	// Called when another command which requires one or more of the same
// 	// subsystems is scheduled to run
// 	protected void interrupted() {
// 		Robot.drivetrain.setUsingTurnPID(false);
// 		Robot.drivetrain.stop();
// 	}
// }