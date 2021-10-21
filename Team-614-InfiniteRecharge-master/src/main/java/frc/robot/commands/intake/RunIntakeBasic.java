package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.feeder.Feeder;

/**
 *
 */
public class RunIntakeBasic extends Command {
    double speed;
	public RunIntakeBasic(double speed) {
        requires(Robot.m_intake);
        this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_intake.intakeMotor.set(0);
		// Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
		// Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
        // Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if(OI.driverController.getTriggerAxis(Hand.kLeft) > 0.5) {
		Robot.m_intake.intakeMotor.set(speed);
		// Robot.m_serializer.serializerMotorA.set(-0.35);
		// Robot.m_serializer.serializerMotorB.set(0.2);
		Robot.m_feeder.changeCounterBasic();
	    } else {
			Robot.m_intake.intakeMotor.set(0);
		// Robot.m_serializer.serializerMotorA.set(0);
		// Robot.m_serializer.serializerMotorB.set(0);
		Robot.m_feeder.feederMotor.set(0);
		}
	}

	// Make this return true when this Command no lo
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_intake.intakeMotor.set(0);
		// Robot.m_serializer.serializerMotorA.set(0);
		// Robot.m_serializer.serializerMotorB.set(0);
		Robot.m_feeder.feederMotor.set(0);
		// Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
		// Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
        // Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonIn);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_intake.intakeMotor.set(0);
		// Robot.m_serializer.serializerMotorA.set(0);
		// Robot.m_serializer.serializerMotorB.set(0);
		Robot.m_feeder.feederMotor.set(0);
		// Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
		// Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
        // Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonIn);
    }
    
    // protected void pistonToggle() {
    //     Robot.m_intake.toggleDoubleSolenoidA();
    //     Robot.m_intake.toggleDoubleSolenoidB();
    // }
}