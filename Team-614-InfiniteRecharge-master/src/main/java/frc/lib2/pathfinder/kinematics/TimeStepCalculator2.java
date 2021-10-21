package frc.lib2.pathfinder.kinematics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.NoSuchElementException;

import frc.lib2.pathfinder.pathCreator.PathGenerator2;
import frc.lib2.pathfinder.pathCreator.SmoothPosition2;
// import frc.lib.pathfinder.robot.OutputCalculator;
import frc.lib2.util.Point2;

public class TimeStepCalculator2 {
   public static ArrayList<Double> timeOutlined = new ArrayList<Double>();
   public static double timeStep = 0;
   public static double smoothWeightB = 0.82;
   public static double dataWeightA = 1 - smoothWeightB;
   public static double tolerance = 0.001;
   public static double pastVelocity = 0;

   public static RobotTracing2 robotPath2;
   public static ArrayList<Double> velocity2, leftDistance2, rightDistance2;

   
   public static void calculateTimeSteps() {
	   for(int i = 1; i < SmoothPosition2.newPathPoints.size(); i++) {
		timeOutlined.add(timeStep);
	    timeStep += SmoothPosition2.newPathPoints.get(i).distance(SmoothPosition2.newPathPoints.get(i-1)) / SmoothVelocity2.smoothedVelocities.get(i); 
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
   
   public static void printAtTimeStep(double timeStep, RobotTracing2 robotPath) {
	   System.out.println("Time Step:" + getNearestTimeStep(timeStep));
	   System.out.println("Velocity:" + SmoothVelocity2.smoothedVelocities.get(getNearestTimeStepIndex(timeStep)));;
	   System.out.println("Distance Left Path:" + KinematicsCalculator2.leftDistance.get(getNearestTimeStepIndex(timeStep)));
	   System.out.println("Distance Right Path:" + KinematicsCalculator2.rightDistance.get(getNearestTimeStepIndex(timeStep)));
	   System.out.println("Heading:" + robotPath.heading.get(getNearestTimeStepIndex(timeStep)));
   }

//     public static void main(String [] args) {
// 	          PathGenerator2.createDataSet();
//        SmoothPosition2.smoothPath(PathGenerator2.finalPoints, SmoothPosition2.dataWeightA, SmoothPosition2.smoothWeightB,
//                SmoothPosition2.tolerance);
//        KinematicsCalculator2.calculuateCurvature();
//        KinematicsCalculator2.calculateVelocities();

//        KinematicsCalculator2.rateLimiter();
//        SmoothVelocity2.smoothVelocity(KinematicsCalculator2.velocity, SmoothVelocity2.dataWeightA,
//                SmoothVelocity2.smoothWeightB, SmoothVelocity2.tolerance);
//        velocity2 = new ArrayList(SmoothVelocity2.smoothVelocity(KinematicsCalculator2.velocity,
//                SmoothVelocity2.dataWeightA, SmoothVelocity2.smoothWeightB, SmoothVelocity2.tolerance));

//        robotPath2 = new RobotTracing2(SmoothPosition2.newPathPoints, 2);
//        robotPath2.leftRight(SmoothPosition2.newPathPoints, 2);

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
