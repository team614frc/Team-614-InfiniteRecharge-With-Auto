package frc.robot.commands.chassis;

import frc.lib3.pathfinder.kinematics.KinematicsCalculator3;
import frc.lib3.pathfinder.kinematics.TimeStepCalculator3;
import frc.robot.Robot;
import frc.lib3.pathfinder.kinematics.SmoothVelocity3;
import frc.lib3.pathfinder.kinematics.RobotTracing3;

public class OutputCalculator3 {
	
	 double kP, kD, kV;
	 double lastError;
	 double distancePerPulse;

	 public static double calcuatedLeftValue;
	 
	 public OutputCalculator3(double kP, double kD, double kV, double wheelDiameterLength, double encoderSpinValue) {
		 this.kP = kP;
		 this.kD = kD;
		 this.kV = kV;
		 
		 lastError = 0;
		 
		 this.distancePerPulse = ((wheelDiameterLength/13) * Math.PI) / encoderSpinValue;
	 }
	 
     public double calculateDefaultOutput(double encoderTick, int i) {
    	 double distanceCovered = (encoderTick * distancePerPulse);
    	 double error = (KinematicsCalculator3.distance.get(i) - distanceCovered);
    	 double calculatedValue = kP * error 
    			                  + 
    			                  kD * ((error-lastError) / TimeStepCalculator3.timeOutlined.get(i))
    			                  +
    			                  kV * SmoothVelocity3.smoothedVelocities.get(i).doubleValue();
    	 
    	 return calculatedValue;
     }
     
     public void setLastError(double error) {
    	lastError = error; 
     }
     
     public double calculateLeftOutput(double encoderTick, int i, RobotTracing3 robotPath) {
    	 double distanceCovered = (encoderTick * distancePerPulse);
    	 double error = (Robot.m_drivetrain.leftDistance3.get(i) - distanceCovered);
         calcuatedLeftValue = kP * error 
        			              + 
        			              kV * Robot.m_drivetrain.velocity3.get(i).doubleValue();
    	 lastError = error;
    	 return calcuatedLeftValue;
     }
     
     public double calculateRightOutput(double encoderTick, int i, RobotTracing3 robotPath) {
    	 double distanceCovered = (encoderTick * distancePerPulse);
    	 double error = (Robot.m_drivetrain.rightDistance3.get(i) - distanceCovered);
    	 double calculatedValue = kP * error 
    			                  +
    			                  kV * Robot.m_drivetrain.velocity3.get(i).doubleValue();
    	 return calculatedValue;
     }
}
