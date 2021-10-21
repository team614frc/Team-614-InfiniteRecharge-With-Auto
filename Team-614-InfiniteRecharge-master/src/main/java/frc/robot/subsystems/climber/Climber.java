package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import com.playingwithfusion.*;

public class Climber extends Subsystem {
    public CANSparkMax climberMotor;
    public CANPIDController climberPIDController;
    public DigitalInput limitSwitch1;
    public DigitalInput limitSwitch2;
    public DigitalInput limitSwitch3;
    public DigitalInput limitSwitch4;
    public DoubleSolenoid motorBrake;

    public static DoubleSolenoid.Value pistonIn = DoubleSolenoid.Value.kReverse;
    public static DoubleSolenoid.Value pistonOut = DoubleSolenoid.Value.kForward;
    // public CANPIDController

    public Climber() {
        climberMotor = new CANSparkMax(RobotMap.climberMotorPort, MotorType.kBrushless);
        climberPIDController = climberMotor.getPIDController();

        climberPIDController.setP(RobotMap.climberPValue);
        climberPIDController.setI(RobotMap.climberIValue);
        climberPIDController.setD(RobotMap.climberDValue);
        climberPIDController.setFF(RobotMap.climberFFValue);
        climberPIDController.setIZone(RobotMap.climberIZValue);

        climberPIDController.setSmartMotionMaxVelocity(RobotMap.climberMaxVel, RobotMap.climberSmartMotionPort);
        climberPIDController.setSmartMotionMinOutputVelocity(RobotMap.climberMinVel, RobotMap.climberSmartMotionPort);
        climberPIDController.setSmartMotionMaxAccel(RobotMap.climberMaxAcc, RobotMap.climberSmartMotionPort);
        climberPIDController.setSmartMotionAllowedClosedLoopError(RobotMap.climberAllowedErr,
                RobotMap.climberSmartMotionPort);
            
        limitSwitch1 = new DigitalInput(RobotMap.limitSwitchPortA);
        limitSwitch2 = new DigitalInput(RobotMap.limitSwitchPortB);
        limitSwitch3 = new DigitalInput(RobotMap.limitSwitchPortC);
        limitSwitch4 = new DigitalInput(RobotMap.limitSwitchPortD);
                
        motorBrake = new DoubleSolenoid(RobotMap.climberPistonPortA, RobotMap.climberPistonPortB);

        // shooterPIDController.setP(RobotMap.shooterPValue);
        // shooterPIDController.setI(RobotMap.shooterIValue);
        // shooterPIDController.setD(RobotMap.shooterDValue);
        // shooterPIDController.setIZone(RobotMap.shooterIZValue);
        // shooterPIDController.setFF(RobotMap.shooterFFValue);
        // shooterPIDController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput);
    }

    @Override
    public void initDefaultCommand() {

    }

    public void createTelemetry() {
        SmartDashboard.putNumber("Climber: P Value", RobotMap.climberPValue);
        SmartDashboard.putNumber("Climber: I Value", RobotMap.climberIValue);
        SmartDashboard.putNumber("Climber: D Value", RobotMap.climberDValue);
        SmartDashboard.putNumber("Climber: FF Values", RobotMap.climberFFValue);
        SmartDashboard.putNumber("Climber: IZ Value", RobotMap.climberIZValue);
    }

    public void updateTelemetry() {
        if (climberPIDController.getP() != SmartDashboard.getNumber("Climber: P Value", RobotMap.climberPValue)) {
            climberPIDController.setP(SmartDashboard.getNumber("Climber: P Value", RobotMap.climberPValue));
        } else if (climberPIDController.getI() != SmartDashboard.getNumber("Climber: I Value",
                RobotMap.climberIValue)) {
            climberPIDController.setI(SmartDashboard.getNumber("Climber: I Value", RobotMap.climberIValue));
        } else if (climberPIDController.getD() != SmartDashboard.getNumber("Climber: D Value",
                RobotMap.climberDValue)) {
            climberPIDController.setD(SmartDashboard.getNumber("Climber: D Value", RobotMap.climberDValue));
        } else if (climberPIDController.getFF() != SmartDashboard.getNumber("Climber: FF Value",
                RobotMap.climberFFValue)) {
            climberPIDController.setFF(SmartDashboard.getNumber("Climber: FF Value", RobotMap.climberFFValue));
        } else if (climberPIDController.getIZone() != SmartDashboard.getNumber("Climber: IZ Value",
                RobotMap.climberIZValue)) {
            climberPIDController.setIZone(SmartDashboard.getNumber("Climber: IZ Value", RobotMap.climberIZValue));
        }
    }

    public DoubleSolenoid.Value getBrake() {
        return motorBrake.get();
    }

    public void setBrake(DoubleSolenoid.Value state) {
        motorBrake.set(state);
    }

    public DoubleSolenoid.Value getBrakeOppositeState(DoubleSolenoid.Value solenoid) {
        if (solenoid.equals(pistonIn)) {
           return pistonOut;
        } else {
           return pistonIn;
        }
    }

    public void toggleBrake() {
        setBrake(getBrakeOppositeState(getBrake()));
    }
  
    public void runTMP(double setpoint) {
        climberPIDController.setReference(setpoint, ControlType.kSmartMotion);
    }

    public void setPowerOutput(double power) {
        climberMotor.set(power);
    }

    public boolean checkLimitOne() {
        return limitSwitch1.get();
    }

    public boolean checkLimitTwo() {
        return limitSwitch2.get();
    }

    public boolean checkLimitThree() {
        return limitSwitch3.get();
    }

    public boolean checkLimitFour() {
        return limitSwitch4.get();
    }

    public boolean checkTop() {
        return limitSwitch1.get() || limitSwitch2.get();
    }

    public boolean checkBottom() {
        return limitSwitch3.get() || limitSwitch4.get();
    }
    
    public void interruptClimberTop(double state) {
        //climber.set(checkTop() ? 0 : state);
        if(checkTop()) {
            climberMotor.set(0);
        }
        else {
            climberMotor.set(state);
        }
    }

    public void interruptClimberBottom(double state) {
        if(checkBottom())
        {
            climberMotor.set(0);
        }
        else {
            climberMotor.set(state);
        }
    }
}
