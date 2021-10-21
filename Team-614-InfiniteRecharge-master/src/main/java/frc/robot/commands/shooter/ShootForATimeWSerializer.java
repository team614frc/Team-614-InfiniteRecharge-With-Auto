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
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ShootForATimeWSerializer extends Command {
	public static Timer timer;
	Timer timer2;
	double time;
	private AHRS navx;
	private PIDController pid;
	private double angle;
  
	private double limelightAngle;

	public ShootForATimeWSerializer(double time) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_intake);
		requires(Robot.m_shooter);
		requires(Robot.m_feeder);
		requires(Robot.m_drivetrain);
		requires(Robot.m_serializer);
	
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
    // Robot.m_limelight.setLED(0);
        Robot.m_limelight.setPipeline(1);
		Robot.m_limelight.setCamMode(0);
		Robot.m_limelight.setLED(0);
		Robot.m_intake.intakeSolenoidA.set(DoubleSolenoid.Value.kForward);
        Robot.m_intake.intakeSolenoidB.set(DoubleSolenoid.Value.kReverse);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_intake.intakeSolenoidA.set(DoubleSolenoid.Value.kForward);
		Robot.m_intake.intakeSolenoidB.set(DoubleSolenoid.Value.kReverse);
		Robot.m_shooter.setShooterReference(4200);
		if(Math.abs(Robot.m_limelight.getX()) < 0.3 && timer.get() > .5) {
	         	Robot.m_shooter.acceleratorMotor.set(.7);
				Robot.m_serializer.serializerMotorA.set(-0.45);
				Robot.m_serializer.serializerMotorB.set(0.3);
				Robot.m_feeder.feederMotor.set(.95);
				
		}
		Robot.m_intake.intakeMotor.set(0.5);
		
	
		Robot.m_drivetrain.arcadeDrive(0.0, pid.calculate(Robot.m_limelight.getX(), 0) < 0 ? pid.calculate(Robot.m_limelight.getX(), 0) - 0.142 : pid.calculate(Robot.m_limelight.getX(), 0) + .142);
		

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return timer.get() > 4;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_shooter.shooterMotor.set(0);
		Robot.m_shooter.acceleratorMotor.set(0);
		Robot.m_feeder.feederMotor.set(0);
		Robot.m_intake.intakeMotor.set(0);
		Robot.m_serializer.serializerMotorA.set(0);
		Robot.m_serializer.serializerMotorB.set(0);
		
		Robot.m_limelight.setPipeline(3);
		Robot.m_limelight.setCamMode(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_shooter.shooterMotor.set(0);
		Robot.m_shooter.acceleratorMotor.set(0);
		Robot.m_feeder.feederMotor.set(0);
		Robot.m_serializer.serializerMotorA.set(0);
		Robot.m_intake.intakeMotor.set(0);
		Robot.m_limelight.setPipeline(3);
		Robot.m_limelight.setCamMode(0);
	}
}
