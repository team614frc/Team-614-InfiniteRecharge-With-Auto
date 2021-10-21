package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
       public CANSparkMax shooterMotor;
       public CANSparkMax intakeMotor;
    //    private CANPIDController pidController;
    //    private CANEncoder encoder;

       public Shooter() {
         shooterMotor = new CANSparkMax(RobotMap.shooterMotorPort, MotorType.kBrushless);
         shooterMotor = new CANSparkMax(RobotMap.intakeMotorPort, MotorType.kBrushless);
        //  pidController = shooterMotor.getPIDController();
        //  encoder = shooterMotor.getEncoder();

        //  pidController.setP(RobotMap.kPValue);
        //  pidController.setI(RobotMap.kIValue);
        //  pidController.setD(RobotMap.kDValue);
        //  pidController.setIZone(RobotMap.kIZValue);
        //  pidController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput);
       }
    
       public void initDefaultCommand() {
         // setDefaultCommand(new VisionProcessing());
       }

    //    public void setShooterSpeed(double setPoint) {
    //       pidController.setReference(setPoint, ControlType.kVelocity);
    //       SmartDashboard.putNumber("Shooter: Process Variable", encoder.getVelocity());
    //    }
       public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
       }
     

       public void setIntakeSpeed(double speed) {
           intakeMotor.set(speed);
       }
}