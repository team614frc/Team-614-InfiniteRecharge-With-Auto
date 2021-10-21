package frc.robot.commands.serializer;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class RunSerializerBasic extends Command {
	public double speed;
	public RunSerializerBasic(double speed) {
		requires(Robot.m_serializer);
		this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_serializer.serializerMotorA.set(0);
        Robot.m_serializer.serializerMotorB.set(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        Robot.m_serializer.runSerializer(speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
        Robot.m_serializer.serializerMotorA.set(0);
        Robot.m_serializer.serializerMotorB.set(0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.m_serializer.serializerMotorA.set(0);
        Robot.m_serializer.serializerMotorB.set(0);
	}
}
