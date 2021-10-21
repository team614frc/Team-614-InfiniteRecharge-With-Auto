package frc.robot.commands.shooter;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DeliverGoalLow extends Command {
	public Timer timer = new Timer();
	public DeliverGoalLow() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_shooter);
		requires(Robot.m_intake);
		requires(Robot.m_feeder);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.m_shooter.shooterMotor.set(0);
		Robot.m_shooter.acceleratorMotor.set(0);
		timer.reset();
		timer.start();
		Robot.m_limelight.setLED(0);
		Robot.m_limelight.setPipeline(1);
		Robot.m_limelight.setCamMode(0);
		// Robot.m_intake.toggleDoubleSolenoidB();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	   Robot.m_shooter.setShooterReference(3500);
	   if(timer.get() > .2 && Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > 3420) {
		   Robot.m_shooter.acceleratorMotor.set(.7);
		   Robot.m_serializer.serializerMotorA.set(-0.45);
			Robot.m_serializer.serializerMotorB.set(0.3);
			Robot.m_feeder.feederMotor.set(0.7);
			Robot.m_intake.intakeMotor.set(0.5);
	   }
	   SmartDashboard.putNumber("Shooter Speed", Robot.m_shooter.shooterMotor.getEncoder().getVelocity());
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_shooter.shooterMotor.set(0);
		Robot.m_shooter.acceleratorMotor.set(0);
		Robot.m_serializer.serializerMotorA.set(0);
		Robot.m_serializer.serializerMotorB.set(0);
		Robot.m_feeder.feederMotor.set(0);
		Robot.m_intake.intakeMotor.set(0);
		Robot.m_feeder.counter = 0;
		Robot.m_limelight.setPipeline(3);
		Robot.m_limelight.setCamMode(0);
		// Robot.m_intake.toggleDoubleSolenoidB();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_shooter.shooterMotor.set(0);
		Robot.m_shooter.acceleratorMotor.set(0);
		Robot.m_serializer.serializerMotorA.set(0);
		Robot.m_serializer.serializerMotorB.set(0);
		Robot.m_feeder.feederMotor.set(0);
		Robot.m_intake.intakeMotor.set(0);
		Robot.m_feeder.counter = 0;
		Robot.m_limelight.setPipeline(3);
		Robot.m_limelight.setCamMode(0);
		// Robot.m_intake.toggleDoubleSolenoidB();
	}
}
