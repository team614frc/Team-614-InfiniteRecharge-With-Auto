package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.lib3.pathfinder.kinematics.TimeStepCalculator3;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

import frc.lib3.util.AngleMath3;
import frc.lib3.util.Point3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.chassis.Drivetrain;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class FollowPath3 extends Command {
    public Timer timer;
    public static ArrayList<Point3> leftPath3, rightPath;
    public static ArrayList<Double> timeOutlined3;
    public static ArrayList<Double> heading;

    public static double gyroHeading;
    public static double desiredHeading;
    public static double angleDifference;
    public static double headingValue;

    public boolean isFinished = false;

    public FollowPath3() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        // Robot.m_limelight.setLED(3);
        heading = new ArrayList(Drivetrain.robotPath3.heading);
        requires(Robot.m_intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.m_drivetrain.leftMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        Robot.m_drivetrain.leftMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);
        Robot.m_drivetrain.rightMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        Robot.m_drivetrain.rightMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);
        Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
        Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
        Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
        Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
        TimeStepCalculator3.calculateTimeSteps();
        timeOutlined3 = new ArrayList(TimeStepCalculator3.timeOutlined);
        heading = new ArrayList(Drivetrain.robotPath3.heading);
        heading.add(0, 0.0);
        timer = new Timer();
        timer.reset();
        timer.start();
        Robot.m_intake.intakeMotor.set(0.6);
        Robot.m_intake.intakeSolenoidA.set(DoubleSolenoid.Value.kForward);
        Robot.m_intake.intakeSolenoidB.set(DoubleSolenoid.Value.kReverse);
        Robot.m_shooter.setShooterReference(RobotMap.highGoalSpeed);
        Robot.m_serializer.serializerMotorA.set(-0.5);
        Robot.m_serializer.serializerMotorB.set(-0.3);
       
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (!isFinished) {
            try {
                calculateHeading();
                Robot.m_drivetrain.leftMotorA.set(-Robot.m_drivetrain.outputCalculator3.calculateLeftOutput(-Robot.m_drivetrain.leftMotorA.getEncoder().getPosition(), TimeStepCalculator3.getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath3) + (headingValue * RobotMap.hValue));
                Robot.m_drivetrain.rightMotorA.set(Robot.m_drivetrain.outputCalculator3.calculateRightOutput(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition(), TimeStepCalculator3.getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath3) + (headingValue * RobotMap.hValue));

                SmartDashboard.putNumber("Output (Left Wheels) 3",
                        Robot.m_drivetrain.outputCalculator3.calculateLeftOutput(
                                Robot.m_drivetrain.leftMotorA.getEncoder().getPosition(),
                                getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath3)
                                - (headingValue * RobotMap.hValue));
                SmartDashboard.putNumber("Output (Right Wheels) 3",
                        -Robot.m_drivetrain.outputCalculator3.calculateRightOutput(
                                -Robot.m_drivetrain.rightMotorA.getEncoder().getPosition(),
                                getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath3)
                                + (headingValue * RobotMap.hValue));
                SmartDashboard.putNumber("Heading 3", getNearestTimeStepIndex(timer.get()));
            } catch (java.lang.IndexOutOfBoundsException E) {
                isFinished = true;
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.m_drivetrain.leftMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        Robot.m_drivetrain.leftMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        Robot.m_drivetrain.rightMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        Robot.m_drivetrain.rightMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        Robot.m_intake.intakeSolenoidA.set(DoubleSolenoid.Value.kReverse);
        Robot.m_intake.intakeSolenoidB.set(DoubleSolenoid.Value.kForward);
        Robot.m_intake.intakeMotor.set(0);
        Robot.m_serializer.serializerMotorA.set(0);
		Robot.m_serializer.serializerMotorB.set(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    public int getNearestTimeStepIndex(double time) {
        int j = 0;

        for (int i = 0; time > timeOutlined3.get(i); i++) {
            j = i;
        }
        return j;
    }

    public void calculateHeading() {
        gyroHeading = Robot.m_navX.getAngle();
        desiredHeading = heading.get(getNearestTimeStepIndex(timer.get()));
        angleDifference = AngleMath3.boundDegrees(desiredHeading - gyroHeading);
        headingValue = angleDifference;
    }
}
