package frc.lib3.pathfinder.kinematics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.NoSuchElementException;

import frc.lib3.pathfinder.pathCreator.PathGenerator3;
import frc.lib3.pathfinder.pathCreator.SmoothPosition3;
// import frc.lib.pathfinder.robot.OutputCalculator;
import frc.lib3.util.Point3;

public class TimeStepCalculator3 {
   public static ArrayList<Double> timeOutlined = new ArrayList<Double>();
   public static double timeStep = 0;
   public static double smoothWeightB = 0.82;
   public static double dataWeightA = 1 - smoothWeightB;
   public static double tolerance = 0.001;
   public static double pastVelocity = 0;

   public static RobotTracing3 robotPath2;
   public static ArrayList<Double> velocity2, leftDistance2, rightDistance2;

   
   public static void calculateTimeSteps() {
	   for(int i = 1; i < SmoothPosition3.newPathPoints.size(); i++) {
		timeOutlined.add(timeStep);
	    timeStep += SmoothPosition3.newPathPoints.get(i).distance(SmoothPosition3.newPathPoints.get(i-1)) / SmoothVelocity3.smoothedVelocities.get(i); 
	   }
   }
   
   public static double getNearestTimeStep(double time) {
	   double c = timeOutlined.stream()
	            .min(Comparator.comparingDouble(i -> Math.abs(i - time)))
	            .orElseThrow(() -> new NoSuchElementException("No value present"));
	   return c;
   }

   public static int getNearestTimeStepIndex(double time) {
	int j = 0;
	for(int i = 0; time > timeOutlined.get(i); i++) {
		j = i;
	}
	return j;
   }
   
   public static void printAtTimeStep(double timeStep, RobotTracing3 robotPath) {
	   System.out.println("Time Step:" + getNearestTimeStep(timeStep));
	   System.out.println("Velocity:" + SmoothVelocity3.smoothedVelocities.get(getNearestTimeStepIndex(timeStep)));;
	   System.out.println("Distance Left Path:" + KinematicsCalculator3.leftDistance.get(getNearestTimeStepIndex(timeStep)));
	   System.out.println("Distance Right Path:" + KinematicsCalculator3.rightDistance.get(getNearestTimeStepIndex(timeStep)));
	   System.out.println("Heading:" + robotPath.heading.get(getNearestTimeStepIndex(timeStep)));
   }

//     public static void main(String [] args) {
// 	          PathGenerator2.createDataSet();
//        SmoothPosition3.smoothPath(PathGenerator2.finalPoints, SmoothPosition3.dataWeightA, SmoothPosition3.smoothWeightB,
//                SmoothPosition3.tolerance);
//        KinematicsCalculator2.calculuateCurvature();
//        KinematicsCalculator2.calculateVelocities();

//        KinematicsCalculator2.rateLimiter();
//        SmoothVelocity2.smoothVelocity(KinematicsCalculator2.velocity, SmoothVelocity2.dataWeightA,
//                SmoothVelocity2.smoothWeightB, SmoothVelocity2.tolerance);
//        velocity2 = new ArrayList(SmoothVelocity2.smoothVelocity(KinematicsCalculator2.velocity,
//                SmoothVelocity2.dataWeightA, SmoothVelocity2.smoothWeightB, SmoothVelocity2.tolerance));

//        robotPath2 = new RobotTracing2(SmoothPosition3.newPathPoints, 2);
//        robotPath2.leftRight(SmoothPosition3.newPathPoints, 2);

//        KinematicsCalculator2.calculateLeftDistance(robotPath2.leftPath);
//        KinematicsCalculator2.calculateRightDistance(robotPath2.rightPath);
//        leftDistance2 = new ArrayList(KinematicsCalculator2.leftDistance);
//        rightDistance2 = new ArrayList(KinematicsCalculator2.rightDistance);
//        KinematicsCalculator2.calculateLeftVelocities(robotPath2.leftPath);
// 	   KinematicsCalculator2.calculateRightVelocities(robotPath2.rightPath); 
// 	   System.out.println("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// 	   for(int i = 0; i < velocity2.size(); i++) {
//             System.out.println(velocity2.get(i));
// 	   }
//     // } 
//    } 
}
