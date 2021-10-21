package frc.robot.commands.serializer;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;


public class SerializerAutomated extends Command {
    public double speed;
    Timer timer;
    public boolean runSerializer = false;
    public boolean setToRun = false;

	public SerializerAutomated(double speed) {
        this.speed = speed;
        timer = new Timer();
		requires(Robot.m_serializer);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        timer.reset();
        timer.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        if(timer.get() > 4) {
          setToRun = true;
        }
        
        if(Robot.m_intake.getCurrent() > 100) {
           timer.reset();
        }

        if(timer.get() < 4 && setToRun) {
            Robot.m_serializer.runMotorFunction(speed);
        } else {
            Robot.m_serializer.serializerMotorA.set(0);
            Robot.m_serializer.serializerMotorB.set(0);
        }
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
