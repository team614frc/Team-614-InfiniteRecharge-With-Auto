package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.lib2.pathfinder.kinematics.TimeStepCalculator2;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

import frc.lib2.util.AngleMath2;
import frc.lib2.util.Point2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.chassis.Drivetrain;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class FollowPath2 extends Command {
    public Timer timer;
    public static ArrayList<Point2> leftPath2, rightPath;
    public static ArrayList<Double> timeOutlined2;
    public static ArrayList<Double> heading;

    public static double gyroHeading;
    public static double desiredHeading;
    public static double angleDifference;
    public static double headingValue;

    public boolean isFinished = false;

    public FollowPath2() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        // Robot.m_limelight.setLED(2);
        heading = new ArrayList(Drivetrain.robotPath2.heading);
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
        TimeStepCalculator2.calculateTimeSteps();
        timeOutlined2 = new ArrayList(TimeStepCalculator2.timeOutlined);
        heading = new ArrayList(Drivetrain.robotPath2.heading);
        heading.add(0, 0.0);
        timer = new Timer();
        timer.reset();
        timer.start();
        Robot.m_intake.intakeMotor.set(0);
        Robot.m_intake.intakeSolenoidA.set(DoubleSolenoid.Value.kReverse);
        Robot.m_intake.intakeSolenoidB.set(DoubleSolenoid.Value.kForward);
        Robot.m_shooter.setShooterReference(RobotMap.highGoalSpeed);
        Robot.m_serializer.serializerMotorA.set(-0.5);
		Robot.m_serializer.serializerMotorB.set(-0.2);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (!isFinished) {
            try {
                calculateHeading();
                Robot.m_drivetrain.leftMotorA.set(Robot.m_drivetrain.outputCalculator2.calculateLeftOutput(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition(), TimeStepCalculator2.getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath2) + (headingValue * RobotMap.hValue));
                Robot.m_drivetrain.rightMotorA.set(-Robot.m_drivetrain.outputCalculator2.calculateRightOutput(-Robot.m_drivetrain.rightMotorA.getEncoder().getPosition(), TimeStepCalculator2.getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath2) + (headingValue * RobotMap.hValue));

                SmartDashboard.putNumber("Output (Left Wheels) 2",
                        Robot.m_drivetrain.outputCalculator2.calculateLeftOutput(
                                Robot.m_drivetrain.leftMotorA.getEncoder().getPosition(),
                                getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath2)
                                - (headingValue * RobotMap.hValue));
                SmartDashboard.putNumber("Output (Right Wheels) 2",
                        -Robot.m_drivetrain.outputCalculator2.calculateRightOutput(
                                -Robot.m_drivetrain.rightMotorA.getEncoder().getPosition(),
                                getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath2)
                                + (headingValue * RobotMap.hValue));
                SmartDashboard.putNumber("Heading 2", getNearestTimeStepIndex(timer.get()));
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

        for (int i = 0; time > timeOutlined2.get(i); i++) {
            j = i;
        }
        return j;
    }

    public void calculateHeading() {
        gyroHeading = Robot.m_navX.getAngle();
        desiredHeading = heading.get(getNearestTimeStepIndex(timer.get()));
        angleDifference = AngleMath2.boundDegrees(desiredHeading - gyroHeading);
        headingValue = angleDifference;
    }
}
