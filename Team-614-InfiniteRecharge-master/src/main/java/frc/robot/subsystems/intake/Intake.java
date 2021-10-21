/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.serializer.Serializer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.intake.IntakeOnJoystick;
import frc.robot.subsystems.compressorcontrol.*;
import frc.robot.commands.intake.RunIntakeBasic;
public class Intake extends Subsystem {
   // Put methods for controlling this subsystem
   // here. Call these from Commands.

   // One Intake Motor, simple speed
   public CANSparkMax intakeMotor;
   public DoubleSolenoid intakeSolenoidA;
   public DoubleSolenoid intakeSolenoidB;

   public DoubleSolenoid.Value pistonIn = DoubleSolenoid.Value.kReverse;
   public DoubleSolenoid.Value pistonOut = DoubleSolenoid.Value.kForward;

   public Intake() {
      intakeMotor = new CANSparkMax(RobotMap.intakeMotorPort, MotorType.kBrushed);
      intakeSolenoidA = new DoubleSolenoid(RobotMap.doubleSolenoidAPort1, RobotMap.doubleSolenoidAPort2);
      intakeSolenoidB = new DoubleSolenoid(RobotMap.doubleSolenoidBPort1, RobotMap.doubleSolenoidBPort2);
      SmartDashboard.putNumber("Intake: Intake Speed", 0.0);
   }

   @Override
   public void initDefaultCommand() {
      setDefaultCommand(new RunIntakeBasic(0.5));
   }

   public DoubleSolenoid.Value getDoubleSolenoidA() {
      return intakeSolenoidA.get();
   }

   public DoubleSolenoid.Value getDoubleSolenoidB() {
      return intakeSolenoidB.get();
   }


   public void setDoubleSolenoidA(DoubleSolenoid.Value state) {
      intakeSolenoidA.set(state);
   }

   public void setDoubleSolenoidB(DoubleSolenoid.Value state) {
      intakeSolenoidB.set(state);
   }

   public DoubleSolenoid.Value getOppositeState(DoubleSolenoid.Value solenoid) {
      if (solenoid.equals(pistonIn)) {
         return pistonOut;
      } else {
         return pistonIn;
      }
   }

   public void toggleDoubleSolenoidA() {
      setDoubleSolenoidA(getOppositeState(getDoubleSolenoidA()));
   }

   public void toggleDoubleSolenoidB() {
      setDoubleSolenoidB(getOppositeState(getDoubleSolenoidB()));
   }

   public void intakemotorPortSpeed(double speed) {
      intakeMotor.set(speed);
   }

   public double getCurrent() {
      return intakeMotor.getOutputCurrent();
   }

   public boolean checkCurrent(double current) {
      return intakeMotor.getOutputCurrent() > current ? true : false;
   }

   public void runSerializer(Serializer serializer, double current, double speed) {
      if (checkCurrent(current)) {
         serializer.runMotorFunction(speed);
      }
   }
}