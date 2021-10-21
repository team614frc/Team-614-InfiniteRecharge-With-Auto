package frc.robot.subsystems.chassis;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.chassis.ArcadeDrive;
import frc.robot.commands.chassis.ModifiedCurvatureDrive;
import frc.robot.commands.chassis.OutputCalculator;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.lib.pathfinder.pathCreator.PathGenerator;
import frc.lib.pathfinder.pathCreator.SmoothPosition;
import frc.lib.pathfinder.kinematics.*;
import frc.lib.pathfinder.kinematics.RobotTracing;
import java.util.ArrayList;
import frc.lib.util.AngleMath;
import frc.robot.Robot;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.chassis.VelocityDrive;
import frc.robot.commands.chassis.ModifiedArcadeDrive;

import frc.lib2.pathfinder.pathCreator.PathGenerator2;
import frc.lib2.pathfinder.pathCreator.SmoothPosition2;
import frc.lib2.pathfinder.kinematics.*;
import frc.lib2.pathfinder.kinematics.RobotTracing2;
import frc.lib2.util.AngleMath2;
import frc.robot.commands.chassis.OutputCalculator2;

import frc.lib3.pathfinder.pathCreator.PathGenerator3;
import frc.lib3.pathfinder.pathCreator.SmoothPosition3;
import frc.lib3.pathfinder.kinematics.*;
import frc.lib3.pathfinder.kinematics.RobotTracing3;
import frc.lib3.util.AngleMath3;
import frc.robot.commands.chassis.OutputCalculator3;

public class Drivetrain extends Subsystem {

    public CANSparkMax leftMotorA, leftMotorB, rightMotorA, rightMotorB;
    public static DifferentialDrive drivetrain;

    public OutputCalculator outputCalculator;
    public static double P, I, D, V;
    public static RobotTracing robotPath;
    public ArrayList<Double> velocity, leftDistance, rightDistance;

    public OutputCalculator2 outputCalculator2;
    public static RobotTracing2 robotPath2;
    public ArrayList<Double> velocity2, leftDistance2, rightDistance2;

    public OutputCalculator3 outputCalculator3;
    public static RobotTracing3 robotPath3;
    public ArrayList<Double> velocity3, leftDistance3, rightDistance3;

    public CANPIDController pidControllerLeft, pidControllerrRight;
    public CANEncoder encoderLeft, encoderRight;

    static final double turnTolerance = 0.1f;


    public Drivetrain() {
        leftMotorA = new CANSparkMax(RobotMap.leftMotorAPort, RobotMap.brushless);
        leftMotorB = new CANSparkMax(RobotMap.leftMotorBPort, RobotMap.brushless);
        rightMotorA = new CANSparkMax(RobotMap.rightMotorAPort, RobotMap.brushless);
        rightMotorB = new CANSparkMax(RobotMap.rightMotorBPort, RobotMap.brushless);

        drivetrain = new DifferentialDrive(leftMotorA, rightMotorA);
        
        leftMotorA.setInverted(true);
        rightMotorA.setInverted(true);
        leftMotorB.follow(leftMotorA);
        rightMotorB.follow(rightMotorA);

        leftMotorA.getPIDController().setP(5e-5);
        leftMotorA.getPIDController().setI(1e-6);
        leftMotorA.getPIDController().setD(0);
        leftMotorA.getPIDController().setIZone(0);
        leftMotorA.getPIDController().setFF(0);
        leftMotorA.getPIDController().setOutputRange(-1, 1);
        
        rightMotorA.getPIDController().setP(5e-6);
        rightMotorA.getPIDController().setI(1e-6);
        rightMotorA.getPIDController().setD(0);
        rightMotorA.getPIDController().setIZone(0);
        rightMotorA.getPIDController().setFF(0);
        rightMotorA.getPIDController().setOutputRange(-1, 1);

        leftMotorA.getPIDController().setSmartMotionMaxVelocity(2000, 0);
        leftMotorA.getPIDController().setSmartMotionMinOutputVelocity(0, 0);
        leftMotorA.getPIDController().setSmartMotionMaxAccel(1000, 0);
        leftMotorA.getPIDController().setSmartMotionAllowedClosedLoopError(0, 0);

        rightMotorA.getPIDController().setSmartMotionMaxVelocity(3000, 0);
        rightMotorA.getPIDController().setSmartMotionMinOutputVelocity(0, 0);
        rightMotorA.getPIDController().setSmartMotionMaxAccel(1000, 0);
        rightMotorA.getPIDController().setSmartMotionAllowedClosedLoopError(0, 0);

        outputCalculator = new OutputCalculator(RobotMap.pValue, RobotMap.dValue, RobotMap.vValue,
                 RobotMap.wheelDiameter, RobotMap.ticksInARevolution);
        PathGenerator.createDataSet();
        SmoothPosition.smoothPath(PathGenerator.finalPoints, SmoothPosition.dataWeightA, SmoothPosition.smoothWeightB,
                SmoothPosition.tolerance);
        KinematicsCalculator.calculuateCurvature();
        KinematicsCalculator.calculateVelocities();

        KinematicsCalculator.rateLimiter();
        SmoothVelocity.smoothVelocity(KinematicsCalculator.velocity, SmoothVelocity.dataWeightA,
                SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
        velocity = new ArrayList(SmoothVelocity.smoothVelocity(KinematicsCalculator.velocity,
                SmoothVelocity.dataWeightA, SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance));

        robotPath = new RobotTracing(SmoothPosition.newPathPoints, 2);
        robotPath.leftRight(SmoothPosition.newPathPoints, 2);

        KinematicsCalculator.calculateLeftDistance(robotPath.leftPath);
        KinematicsCalculator.calculateRightDistance(robotPath.rightPath);
        leftDistance = new ArrayList(KinematicsCalculator.leftDistance);
        rightDistance = new ArrayList(KinematicsCalculator.rightDistance);
        KinematicsCalculator.calculateLeftVelocities(robotPath.leftPath);
        KinematicsCalculator.calculateRightVelocities(robotPath.rightPath);
        SmartDashboard.putNumber("Drivetrain: Heading ", leftDistance.size());

        SmoothVelocity.smoothLeftVelocity(KinematicsCalculator.leftVelocity, SmoothVelocity.dataWeightA,
                SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
        SmoothVelocity.smoothRightVelocity(KinematicsCalculator.rightVelocity, SmoothVelocity.dataWeightA,
                SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);

                outputCalculator2 = new OutputCalculator2(RobotMap.pValue, RobotMap.dValue, RobotMap.vValue,
                RobotMap.wheelDiameter, RobotMap.ticksInARevolution);
        
               PathGenerator2.createDataSet();
               SmoothPosition2.smoothPath(PathGenerator2.finalPoints, SmoothPosition2.dataWeightA, SmoothPosition2.smoothWeightB,
                       SmoothPosition2.tolerance);
               KinematicsCalculator2.calculuateCurvature();
               KinematicsCalculator2.calculateVelocities();
        
               KinematicsCalculator2.rateLimiter();
               SmoothVelocity2.smoothVelocity(KinematicsCalculator2.velocity, SmoothVelocity2.dataWeightA,
                       SmoothVelocity2.smoothWeightB, SmoothVelocity2.tolerance);
               velocity2 = new ArrayList(SmoothVelocity2.smoothVelocity(KinematicsCalculator2.velocity,
                       SmoothVelocity2.dataWeightA, SmoothVelocity2.smoothWeightB, SmoothVelocity2.tolerance));
        
               robotPath2 = new RobotTracing2(SmoothPosition2.newPathPoints, 2);
               robotPath2.leftRight(SmoothPosition2.newPathPoints, 2);
        
               KinematicsCalculator2.calculateLeftDistance(robotPath2.leftPath);
               KinematicsCalculator2.calculateRightDistance(robotPath2.rightPath);
               leftDistance2 = new ArrayList(KinematicsCalculator2.leftDistance);
               rightDistance2 = new ArrayList(KinematicsCalculator2.rightDistance);
               KinematicsCalculator2.calculateLeftVelocities(robotPath2.leftPath);
               KinematicsCalculator2.calculateRightVelocities(robotPath2.rightPath);
               SmartDashboard.putNumber("Drivetrain: Heading ", leftDistance2.size());
        
               SmoothVelocity2.smoothLeftVelocity(KinematicsCalculator2.leftVelocity, SmoothVelocity2.dataWeightA,
                       SmoothVelocity2.smoothWeightB, SmoothVelocity2.tolerance);
               SmoothVelocity2.smoothRightVelocity(KinematicsCalculator.rightVelocity, SmoothVelocity.dataWeightA,
                       SmoothVelocity2.smoothWeightB, SmoothVelocity2.tolerance);

                       outputCalculator3 = new OutputCalculator3(RobotMap.pValue, RobotMap.dValue, RobotMap.vValue,
                RobotMap.wheelDiameter, RobotMap.ticksInARevolution);
        
               PathGenerator3.createDataSet();
               SmoothPosition3.smoothPath(PathGenerator3.finalPoints, SmoothPosition3.dataWeightA, SmoothPosition3.smoothWeightB,
                       SmoothPosition3.tolerance);
               KinematicsCalculator3.calculuateCurvature();
               KinematicsCalculator3.calculateVelocities();
        
               KinematicsCalculator3.rateLimiter();
               SmoothVelocity3.smoothVelocity(KinematicsCalculator3.velocity, SmoothVelocity3.dataWeightA,
                       SmoothVelocity3.smoothWeightB, SmoothVelocity3.tolerance);
               velocity3 = new ArrayList(SmoothVelocity3.smoothVelocity(KinematicsCalculator3.velocity,
                       SmoothVelocity3.dataWeightA, SmoothVelocity3.smoothWeightB, SmoothVelocity3.tolerance));
        
               robotPath3 = new RobotTracing3(SmoothPosition3.newPathPoints, 2);
               robotPath3.leftRight(SmoothPosition3.newPathPoints, 2);
        
               KinematicsCalculator3.calculateLeftDistance(robotPath3.leftPath);
               KinematicsCalculator3.calculateRightDistance(robotPath3.rightPath);
               leftDistance3 = new ArrayList(KinematicsCalculator3.leftDistance);
               rightDistance3 = new ArrayList(KinematicsCalculator3.rightDistance);
               KinematicsCalculator3.calculateLeftVelocities(robotPath3.leftPath);
               KinematicsCalculator3.calculateRightVelocities(robotPath3.rightPath);
               SmartDashboard.putNumber("Drivetrain: Heading ", leftDistance3.size());
        
               SmoothVelocity3.smoothLeftVelocity(KinematicsCalculator3.leftVelocity, SmoothVelocity3.dataWeightA,
                       SmoothVelocity3.smoothWeightB, SmoothVelocity3.tolerance);
               SmoothVelocity3.smoothRightVelocity(KinematicsCalculator.rightVelocity, SmoothVelocity.dataWeightA,
                       SmoothVelocity3.smoothWeightB, SmoothVelocity3.tolerance);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ModifiedArcadeDrive());
    }

    public void resetPath() {
        PathGenerator.newPoints.clear();
        PathGenerator.newVectors.clear();
        PathGenerator.finalPoints.clear();
        PathGenerator.newNumOPoints.clear();
        SmoothPosition.newPathPoints.clear();
        SmoothPosition.pathPoints.clear();
        KinematicsCalculator.curvature.clear();
        KinematicsCalculator.distance.clear();
        KinematicsCalculator.leftDistance.clear();
        KinematicsCalculator.leftVelocity.clear();
        KinematicsCalculator.outputs.clear();
        KinematicsCalculator.rightDistance.clear();
        KinematicsCalculator.rightVelocity.clear();
        KinematicsCalculator.velocity.clear();
        SmoothVelocity.leftVelocities.clear();
        SmoothVelocity.rightVelocities.clear();
        TimeStepCalculator.timeOutlined.clear();
        velocity.clear();
        leftDistance.clear();
        rightDistance.clear();
    }

    public void addPoint(double xValue, double yValue) {
        PathGenerator.addPoint(xValue, yValue);
    }

    public void generatePath() {
        outputCalculator = new OutputCalculator(RobotMap.pValue, RobotMap.dValue, RobotMap.vValue,
                RobotMap.wheelDiameter, RobotMap.ticksInARevolution);

        PathGenerator.createDataSet();
        SmoothPosition.smoothPath(PathGenerator.finalPoints, SmoothPosition.dataWeightA, SmoothPosition.smoothWeightB,
                SmoothPosition.tolerance);
        KinematicsCalculator.calculuateCurvature();
        KinematicsCalculator.calculateVelocities();

        KinematicsCalculator.rateLimiter();
        SmoothVelocity.smoothVelocity(KinematicsCalculator.velocity, SmoothVelocity.dataWeightA,
                SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
        velocity = new ArrayList(SmoothVelocity.smoothVelocity(KinematicsCalculator.velocity,
                SmoothVelocity.dataWeightA, SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance));
        robotPath = new RobotTracing(SmoothPosition.newPathPoints, 2);
        robotPath.leftRight(SmoothPosition.newPathPoints, 2);

        KinematicsCalculator.calculateLeftDistance(robotPath.leftPath);
        KinematicsCalculator.calculateRightDistance(robotPath.rightPath);
        leftDistance = new ArrayList(KinematicsCalculator.leftDistance);
        rightDistance = new ArrayList(KinematicsCalculator.rightDistance);
        KinematicsCalculator.calculateLeftVelocities(robotPath.leftPath);
        KinematicsCalculator.calculateRightVelocities(robotPath.rightPath);

        SmoothVelocity.smoothLeftVelocity(KinematicsCalculator.leftVelocity, SmoothVelocity.dataWeightA,
                SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
        SmoothVelocity.smoothRightVelocity(KinematicsCalculator.rightVelocity, SmoothVelocity.dataWeightA,
                SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
    }

    public void arcadeDrive(double speed, double rotateValue) {
        drivetrain.arcadeDrive(speed, rotateValue);
    }

    public void curvatureDrive(double speed, double rotateValue) {
        drivetrain.curvatureDrive(speed, rotateValue, OI.driverController.getAButton());
    }

    public double distanceInFeet(double encoderValue) {
        return encoderValue * (((RobotMap.wheelDiameter / 12) * Math.PI) / RobotMap.ticksInARevolution);
    }

    public void velocityBasedDrive(XboxController m_driverController) {
        SmartDashboard.putNumber("Left Motor A", leftMotorA.getEncoder().getVelocity());
        if ((m_driverController.getY(Hand.kLeft) > 0.05)
                && (m_driverController.getX(Hand.kRight) > 0.05 || m_driverController.getX(Hand.kRight) < -0.05)) {
            double setPointLeft = Math.sqrt(Math.abs(m_driverController.getY(Hand.kLeft)))
                    * m_driverController.getY(Hand.kLeft) * 4000
                    + Math.sqrt(Math.abs(m_driverController.getX(Hand.kRight))) * m_driverController.getX(Hand.kRight)
                            * 3800;
            double setPointRight = Math.sqrt(Math.abs(m_driverController.getY(Hand.kLeft)))
                    * m_driverController.getY(Hand.kLeft) * 4000
                    - Math.sqrt(Math.abs(m_driverController.getX(Hand.kRight))) * m_driverController.getX(Hand.kRight)
                            * 3800;
            // double setPointLeft = m_driverController.getY(Hand.kLeft)*650;
            // double setPointRight = m_driverController.getY(Hand.kLeft)*650;
            leftMotorA.getPIDController().setReference(setPointLeft, ControlType.kVelocity);
            rightMotorA.getPIDController().setReference(setPointRight, ControlType.kVelocity);

        } else if ((m_driverController.getY(Hand.kLeft) < -0.05)
                && (m_driverController.getX(Hand.kRight) > 0.05 || m_driverController.getX(Hand.kRight) < -0.05)) {
            double setPointLeft = Math.sqrt(Math.abs(m_driverController.getY(Hand.kLeft)))
                    * m_driverController.getY(Hand.kLeft) * 4000
                    - Math.sqrt(Math.abs(m_driverController.getX(Hand.kRight))) * m_driverController.getX(Hand.kRight)
                            * 3800;
            double setPointRight = Math.sqrt(Math.abs(m_driverController.getY(Hand.kLeft)))
                    * m_driverController.getY(Hand.kLeft) * 4000
                    + Math.sqrt(Math.abs(m_driverController.getX(Hand.kRight))) * m_driverController.getX(Hand.kRight)
                            * 3800;
            // double setPointLeft = m_driverController.getY(Hand.kLeft)*650;
            // double setPointRight = m_driverController.getY(Hand.kLeft)*650;
            leftMotorA.getPIDController().setReference(setPointLeft, ControlType.kVelocity);
            rightMotorA.getPIDController().setReference(setPointRight, ControlType.kVelocity);
           

        } else if ((m_driverController.getY(Hand.kLeft) > 0.05 || m_driverController.getY(Hand.kLeft) < -0.05)) {
            double setPointLeft = Math.sqrt(Math.abs(m_driverController.getY(Hand.kLeft)))
                    * m_driverController.getY(Hand.kLeft) * 4000;
            double setPointRight = Math.sqrt(Math.abs(m_driverController.getY(Hand.kLeft)))
                    * m_driverController.getY(Hand.kLeft) * 4000;
            // double setPointLeft = m_driverController.getY(Hand.kLeft)*650;
            // double setPointRight = m_driverController.getY(Hand.kLeft)*650;
            leftMotorA.getPIDController().setReference(setPointLeft, ControlType.kVelocity);
            rightMotorA.getPIDController().setReference(setPointRight, ControlType.kVelocity);

        } else if ((m_driverController.getX(Hand.kRight) > 0.05 || m_driverController.getX(Hand.kRight) < -0.05)) {
            double setPointLeft = -Math.sqrt(Math.abs(m_driverController.getX(Hand.kRight)))
                    * m_driverController.getX(Hand.kRight) * 3800;
            double setPointRight = Math.sqrt(Math.abs(m_driverController.getX(Hand.kRight)))
                    * m_driverController.getX(Hand.kRight) * 3800;
            // double setPointLeft = m_driverController.getY(Hand.kLeft)*650;
            // double setPointRight = m_driverController.getY(Hand.kLeft)*650;
            leftMotorA.getPIDController().setReference(setPointLeft, ControlType.kVelocity);
            rightMotorA.getPIDController().setReference(setPointRight, ControlType.kVelocity);
        } else {
            leftMotorA.set(0);
            rightMotorA.set(0);
        }
    }

    public void resetDrivetrain() {
        Robot.m_navX.reset();
        leftMotorA.set(0);
        leftMotorB.set(0);
        rightMotorA.set(0);
        rightMotorB.set(0);
    }

    public double getAngle() {
        return AngleMath.boundDegrees(Robot.m_navX.getAngle());
    }
}