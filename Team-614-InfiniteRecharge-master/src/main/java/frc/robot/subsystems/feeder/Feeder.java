package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.*;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.ControlType;
import frc.robot.subsystems.shooter.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import com.revrobotics.CANEncoder;
import frc.robot.commands.feeder.*;

public class Feeder extends Subsystem {
    public CANSparkMax feederMotor;
    private CANPIDController feederPIDController;
    public TimeOfFlight tofSensorFeeder;

    public AddressableLED feederLED;
    public AddressableLEDBuffer feederLEDBuffer;
    private CANEncoder encoder;

    public int counter = 0;

    public boolean senseBallFeeder = false;
    public boolean senseBallShooter = false;

    public double distanceFeeder = 0.0;
    public double distanceShooter = 0.0;

    public Timer timer = new Timer();

    public Feeder() {
        feederMotor = new CANSparkMax(RobotMap.feederMotorPort, MotorType.kBrushless);
        feederPIDController = feederMotor.getPIDController();
        tofSensorFeeder = new TimeOfFlight(0); // inside the robot
        Robot.m_shooter.tofSensorShooter.getRange(); // outside the robot

        encoder = feederMotor.getEncoder();
        feederLED = new AddressableLED(0);
        feederLEDBuffer = new AddressableLEDBuffer(60);

        feederLED.setLength(feederLEDBuffer.getLength());

        // Set the data
        feederLED.setData(feederLEDBuffer);

        // PID trapezoidal stuff
        feederPIDController.setP(RobotMap.feederPValue);
        feederPIDController.setI(RobotMap.feederIValue);
        feederPIDController.setD(RobotMap.feederDValue);
        feederPIDController.setIZone(RobotMap.feederIZValue);
        feederPIDController.setFF(RobotMap.feederFFValue);
        feederPIDController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput); // min & max not setup for feeder
                                                                                    // specifically, using shooter's
    }

    @Override
    public void initDefaultCommand() {
        // setDefaultCommand(new FeederAutomated());
    }

    public void createTelemetryFeeder() {
        SmartDashboard.putNumber("Feeder: P Value", RobotMap.feederPValue);
        SmartDashboard.putNumber("Feeder: I Value", RobotMap.feederIValue);
        SmartDashboard.putNumber("Feeder: D Value", RobotMap.feederDValue);
        SmartDashboard.putNumber("Feeder: FF Values", RobotMap.feederFFValue);
        SmartDashboard.putNumber("Feeder: IZ Value", RobotMap.feederIZValue);
    }

    public void updateTelemetryFeeder() {
        if (feederPIDController.getP() != SmartDashboard.getNumber("Feeder: P Value", RobotMap.feederPValue)) {
            feederPIDController.setP(SmartDashboard.getNumber("Feeder: P Value", RobotMap.feederPValue));
        } else if (feederPIDController.getI() != SmartDashboard.getNumber("Feeder: I Value", RobotMap.feederIValue)) {
            feederPIDController.setI(SmartDashboard.getNumber("Feeder: I Value", RobotMap.feederIValue));
        } else if (feederPIDController.getD() != SmartDashboard.getNumber("Feeder: D Value", RobotMap.feederDValue)) {
            feederPIDController.setD(SmartDashboard.getNumber("Feeder: D Value", RobotMap.feederDValue));
        } else if (feederPIDController.getFF() != SmartDashboard.getNumber("Feeder: FF Value",
                RobotMap.feederFFValue)) {
            feederPIDController.setFF(SmartDashboard.getNumber("Feeder: FF Value", RobotMap.feederFFValue));
        } else if (feederPIDController.getIZone() != SmartDashboard.getNumber("Feeder: IZ Value",
                RobotMap.feederIZValue)) {
            feederPIDController.setIZone(SmartDashboard.getNumber("Feeder: IZ Value", RobotMap.feederIZValue));
        }
    }

    public void runTMP(double setpoint) {
        feederPIDController.setReference(setpoint, ControlType.kSmartMotion);
    }

    public void startLED() {
        feederLED.start();
    }

    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }

    public void printTOFSensorReading() {
        SmartDashboard.putNumber("TOF Sensor", tofSensorFeeder.getRange());
    }

    public void runFeederBasic() {
        distanceFeeder = tofSensorFeeder.getRange();
        if(distanceFeeder < 190) {
           setFeederSpeed(0.21);          
        } else {
            setFeederSpeed(0);   
        }
    }

    public void changeCounterBasic() {
        distanceFeeder = tofSensorFeeder.getRange();
        distanceShooter = Robot.m_shooter.tofSensorShooter.getRange();
            if (distanceFeeder < 190 && counter <= 2) {
                if (!senseBallFeeder)
                    counter++;
                senseBallFeeder = true;
                setFeederSpeed(0.18);
            } else {
                senseBallFeeder = false;
                Robot.m_feeder.feederMotor.set(0);		
            }

            // if(counter == 0)
            // {
            //     rainbow();
            // }
    }

    public void changeLED() {
        for(int i = 0; i < (int)(60 * counter/3); i++)
        {
            feederLEDBuffer.setHSV(i, 60, 255, 255);
        }
        for(int i = (int)(60 * counter/3); i < feederLEDBuffer.getLength(); i++)
        {
            feederLEDBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void onLED() {
        for(int i = 0; i < feederLEDBuffer.getLength(); i++)
        {
            feederLEDBuffer.setHSV(i, 60, 255, 255);
        }
    }

    public void offLED() {
        for (var i = 0; i < feederLEDBuffer.getLength(); i++) {
    
          feederLEDBuffer.setRGB(i, 0, 0, 0); // sets it off
    
        }
      }
    
      public void rainbow() {
        if (timer.get() < .05) {
          onLED();
        } else if (timer.get() > .05 && timer.get() < .1) {
          offLED();
        } else {
          timer.reset();
        }
        feederLED.setData(feederLEDBuffer);
      }
    

    public void changeCounterAdvance(double setpoint) {
        distanceFeeder = tofSensorFeeder.getRange();
        distanceShooter = Robot.m_shooter.tofSensorShooter.getRange();
        if (distanceFeeder < 200 && counter <= 3) {
            if (!senseBallFeeder) {
                counter++;
                runTMP(setpoint);
            }
            senseBallFeeder = true;
        } else {
            senseBallFeeder = false;
        }
        if (distanceFeeder < 200 && counter >= 0 && !senseBallFeeder) {
            if (!senseBallShooter) {
                counter++;
            }
            senseBallShooter = true;
        } else {
            senseBallShooter = false;
        }

    }

    public boolean ballInRange() {
        double distance = tofSensorFeeder.getRange();
        boolean balls = false;

        if (distance < 200) {
            balls = true;
        }
        return balls;
    }

}