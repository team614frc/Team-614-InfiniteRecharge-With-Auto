package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightTurnToAngle extends Command {
    private Timer timer;
    private PIDController pid;
    private double angle;

    // Robot.m_limelight.getX(); // < - use this

    public LimelightTurnToAngle(double angle) {
        timer = new Timer();
        timer.start();
        this.angle = angle;

        pid = new PIDController(5e-5, 1e-6, 0);
        pid.setSetpoint(angle);
        pid.setTolerance(0.1f);

        Robot.m_limelight.setPipeline(1);
		Robot.m_limelight.setCamMode(0);
        requires(Robot.m_drivetrain);
        
    }
    
    public void initialize() {
        pid.reset();
    }

    public void execute() {
        double val = pid.calculate(Robot.m_limelight.getX(), angle);
       
        Robot.m_drivetrain.arcadeDrive(0.0, -pid.calculate(Robot.m_limelight.getX(), angle));
        SmartDashboard.putNumber("Value", pid.calculate(Robot.m_limelight.getX(), angle));
        // if (Robot.m_drivetrain.leftMotorA.getEncoder().getVelocity() > 0.4
        //         || Robot.m_drivetrain.rightMotorA.getEncoder().getVelocity() > 0.4) {
        //     timer.reset();
        //     timer.start();
        // }
    }

    public boolean isFinished() {
        return false;
    }

    public void end() {
        Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
        Robot.m_limelight.setPipeline(2);
		Robot.m_limelight.setCamMode(0);
    }

    public void interrupted() {
        Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
        Robot.m_limelight.setPipeline(2);
		Robot.m_limelight.setCamMode(0);
    }
}