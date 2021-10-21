package frc.robot.commands.chassis;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.chassis.Drivetrain;
import edu.wpi.first.wpilibj.Timer;

public class DriveForADistance extends Command {
    public Timer timer;
	
	public DriveForADistance() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_drivetrain);
		requires(Robot.m_limelight);
		// requires(Robot.arm);


		timer = new Timer();
        timer.start();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.m_drivetrain.leftMotorA.set(0);
		Robot.m_drivetrain.rightMotorA.set(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_drivetrain.leftMotorA.getPIDController().setReference(-62.625, ControlType.kSmartMotion);
		Robot.m_drivetrain.rightMotorA.getPIDController().setReference(62.625, ControlType.kSmartMotion);
		if (Robot.m_drivetrain.leftMotorA.getEncoder().getVelocity() > 0.04
                || Robot.m_drivetrain.rightMotorA.getEncoder().getVelocity() > 0.04) {
            timer.reset();
            timer.start();
        }
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_drivetrain.leftMotorA.set(0);
		Robot.m_drivetrain.rightMotorA.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_drivetrain.leftMotorA.set(0);
		Robot.m_drivetrain.rightMotorA.set(0);
	}
}