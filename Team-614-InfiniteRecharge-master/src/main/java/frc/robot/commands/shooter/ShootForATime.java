package frc.robot.commands.shooter;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootForATime extends Command {
	public static Timer timer;
	Timer timer2;
	double time;
	private AHRS navx;
	private PIDController pid;
	private double angle;
  
	private double limelightAngle;

	public ShootForATime(double time) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_shooter);
		requires(Robot.m_feeder);
		// requires(Robot.m_drivetrain);
		timer = new Timer();
		timer2 = new Timer();
	
		pid = new PIDController(RobotMap.sumPValue, RobotMap.iValue, RobotMap.sumDValue);
		pid.setTolerance(0.01f);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		timer.reset();
		timer2.reset();
		timer.start();
		timer2.start();
		pid.reset();
        angle = Robot.m_limelight.getX();
		Robot.m_limelight.setLED(0);
	 Robot.m_limelight.setPipeline(1);
	 Robot.m_limelight.setCamMode(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_shooter.setShooterReference(4200);
		if (timer2.get() > 1.5) {
			Robot.m_shooter.acceleratorMotor.set(.7);
			if (Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > 4000) {
				Robot.m_feeder.feederMotor.set(.6);
			}
		}

		
		
		SmartDashboard.putNumber("Value", pid.calculate(Robot.m_limelight.getX(), 0));
		// double val = pid.calculate(navx.getYaw(), angle);
		// Robot.m_drivetrain.arcadeDrive(0.0, pid.calculate(Robot.m_limelight.getX(), 0) < 0 ? pid.calculate(Robot.m_limelight.getX(), 0) - 0.142 : pid.calculate(Robot.m_limelight.getX(), 0) + .142);
		// if (Robot.m_limelight.getX() != 0) {
		// 	double c = Robot.m_limelight.getX() < 0 ? -0.35 : 0.35;
		// 	double forward = Math.abs(Robot.m_limelight.getDistance() * 0.0035) + 0.35;
		// 	Robot.m_drivetrain.arcadeDrive(0.0, ((Robot.m_limelight.getX() * 0.023) + c));
		// } else {
		// 	Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
		// }

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return timer.get() > 3;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_shooter.shooterMotor.set(0);
		Robot.m_shooter.acceleratorMotor.set(0);
		Robot.m_feeder.feederMotor.set(0);
		Robot.m_navX.reset();
		Robot.m_limelight.setPipeline(3);
		Robot.m_limelight.setCamMode(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_shooter.shooterMotor.set(0);
		Robot.m_shooter.acceleratorMotor.set(0);
		Robot.m_feeder.feederMotor.set(0);
		Robot.m_limelight.setPipeline(3);
		Robot.m_limelight.setCamMode(0);
	}
}
