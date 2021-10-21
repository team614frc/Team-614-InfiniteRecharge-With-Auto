package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder extends Subsystem {
       public CANSparkMax feederMotor= new CANSparkMax(RobotMap.feederMotorPort, MotorType.kBrushed);
       private CANPIDController pidController;
       private CANEncoder encoder;
    
       public void initDefaultCommand() {
         // setDefaultCommand(new VisionProcessing());
       }

       public void set(double setSpeed) {
         feederMotor.set(setSpeed);
       }
}