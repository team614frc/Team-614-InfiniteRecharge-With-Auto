package frc.robot.commands.chassis;

import frc.lib2.pathfinder.kinematics.KinematicsCalculator2;
import frc.lib2.pathfinder.kinematics.TimeStepCalculator2;
import frc.robot.Robot;
import frc.lib2.pathfinder.kinematics.SmoothVelocity2;
import frc.lib2.pathfinder.kinematics.RobotTracing2;

public class OutputCalculator2 {
	
	 double kP, kD, kV;
	 double lastError;
	 double distancePerPulse;

	 public static double calcuatedLeftValue;
	 
	 public OutputCalculator2(double kP, double kD, double kV, double wheelDiameterLength, double encoderSpinValue) {
		 this.kP = kP;
		 this.kD = kD;
		 this.kV = kV;
		 
		 lastError = 0;
		 
		 this.distancePerPulse = ((wheelDiameterLength/12) * Math.PI) / encoderSpinValue;
	 }
	 
     public double calculateDefaultOutput(double encoderTick, int i) {
    	 double distanceCovered = (encoderTick * distancePerPulse);
    	 double error = (KinematicsCalculator2.distance.get(i) - distanceCovered);
    	 double calculatedValue = kP * error 
    			                  + 
    			                  kD * ((error-lastError) / TimeStepCalculator2.timeOutlined.get(i))
    			                  +
    			                  kV * SmoothVelocity2.smoothedVelocities.get(i).doubleValue();
    	 
    	 return calculatedValue;
     }
     
     public void setLastError(double error) {
    	lastError = error; 
     }
     
     public double calculateLeftOutput(double encoderTick, int i, RobotTracing2 robotPath) {
    	 double distanceCovered = (encoderTick * distancePerPulse);
    	 double error = (Robot.m_drivetrain.leftDistance2.get(i) - distanceCovered);
         calcuatedLeftValue = kP * error 
        			              + 
        			              kV * Robot.m_drivetrain.velocity2.get(i).doubleValue();
    	 lastError = error;
    	 return calcuatedLeftValue;
     }
     
     public double calculateRightOutput(double encoderTick, int i, RobotTracing2 robotPath) {
    	 double distanceCovered = (encoderTick * distancePerPulse);
    	 double error = (Robot.m_drivetrain.rightDistance2.get(i) - distanceCovered);
    	 double calculatedValue = kP * error 
    			                  +
    			                  kV * Robot.m_drivetrain.velocity2.get(i).doubleValue();
    	 return calculatedValue;
     }
}
