package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.commands.serializer.*;
//import frc.robot.commands.serializer.SerializerAutomated;
public class Serializer extends Subsystem {
  public CANSparkMax serializerMotorA;
  public CANSparkMax serializerMotorB;

  public Serializer() {
    serializerMotorA = new CANSparkMax(RobotMap.serializerMotorPortA, MotorType.kBrushless);
    serializerMotorB = new CANSparkMax(RobotMap.serializerMotorPortB, MotorType.kBrushless);
  }

  public void initDefaultCommand() {
    // setDefaultCommand(new SerializerAutomated(0.8));
  }

  public void setSerializerA(double speed) {
    serializerMotorA.set(speed);
  }

  public void setSerializerB(double speed) {
    serializerMotorB.set(speed);
  }

  public void runSerializer(double speed) {
    setSerializerA(speed);
    setSerializerB(speed < 0 ? -Math.abs(speed - .2) : Math.abs(speed - .2));
  }

  public double getCurrentA() {
    return serializerMotorA.getOutputCurrent();
  }

  public double getcurrentB() {
    return serializerMotorB.getOutputCurrent();
  }

  public boolean checkCurrentA(double current) {
    return serializerMotorA.getOutputCurrent() > current ? true : false;
  }

  public boolean checkCurrentB(double current) {
    return serializerMotorB.getOutputCurrent() > current ? true : false;
  }

  public void runMotorFunction(double speed) {
       if(!checkCurrentA(RobotMap.setCurrent) && !checkCurrentB(RobotMap.setCurrent))
       {
        setSerializerA(speed);
        setSerializerB(speed < 0 ? -Math.abs(speed - .2) : Math.abs(speed - .2));
       }
       else if(checkCurrentA(RobotMap.setCurrent))
       {
         setSerializerA(-speed);
         setSerializerB(speed < 0 ? -Math.abs(speed - .2) : Math.abs(speed - .2));
       }
       else
       {
         setSerializerB(speed);
         setSerializerA(speed < 0 ? Math.abs(speed - .2) : -Math.abs(speed - .2));
       }
  }
}
