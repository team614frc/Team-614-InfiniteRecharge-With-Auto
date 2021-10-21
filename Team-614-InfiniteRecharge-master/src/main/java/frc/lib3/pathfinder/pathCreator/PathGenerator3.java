
package frc.lib3.pathfinder.pathCreator;



import java.awt.Color;

import java.awt.Graphics;

import java.awt.Graphics2D;

import java.awt.RenderingHints;

import java.awt.Stroke;

import java.util.ArrayList;

import java.util.List;



import frc.lib3.util.Point3;

import frc.lib3.util.Vector3;



public class PathGenerator3 {

	private static final long serialVersionUID = 1L;

	   public static double spacing = 0.5;

	   public static ArrayList<Point3> newPoints = new ArrayList<Point3>();

	   public static ArrayList<Vector3> newVectors = new ArrayList<Vector3>();

	   public static ArrayList<Integer> newNumOPoints = new ArrayList<Integer>();

	   public static ArrayList<Point3> finalPoints = new ArrayList<Point3>();

	   

	   public static void addPoint(double x, double y) {

		newPoints.add(new Point3(x, y));

		}

	   

	   public static void resetPoints() {

		//		   newPoints.remov

}

	   /** 

* 

	 */

	 

	 // Testing Variables

//	   System.out.println("Points:");

//	   for(int i = 0; i < newPoints.size(); i++) {

	//		   System.out.println(i + " X:" + newPoints.get(i).getX());

//		   System.out.println(i + " Y:" + newPoints.get(i).getY());

//	   }



//	   System.out.println("Vectors:");

//	   for(int i = 0; i < newVectors.size(); i++) {
	
	//	      System.out.println(i +" Line Segment Vector In X Direction:" + newVectors.get(i).getX());

//	      System.out.println(i +" Line Segment Vector In Y Direction:" + newVectors.get(i).getY());

//       }



public static void createDataSet(){

	//Add some points to the array

	addPoint(0, 0);
	addPoint(4, 0);

	// addPoint(8, 14);
	// addPoint(9, 14);
	// addPoint(11.5, 9);
	// addPoint(11.5 + 14, 9);
	// addPoint(0, 0);
	// addPoint(4.5, 0);
	// addPoint(4.5, -6.5);
	// addPoint(14, -6.5);

	// addPoint(11.5, 3.4);
	//     addPoint(6.8, 3.4);
	//     addPoint(6.8, 9);
	//     addPoint(29.5, 9);
	//     addPoint(16.5, 4);
	//     addPoint(11.5, 3.4);
	// addPoint(4, 3.32);

	// addPoint(7, 3.32);

	   //Add (points.length - 1) vectors to appropriate the slope 
	   for(int i = 0; i < newPoints.size() - 1; i++) {
		  newVectors.add(new Vector3(newPoints.get(i+1).getX() - newPoints.get(i).getX(), newPoints.get(i+1).getY() - newPoints.get(i).getY()));
	   }

	   //Check for how much points you want 
	   for(int i = 0; i < newVectors.size(); i++) {
		   newNumOPoints.add(new Integer((int) Math.ceil(newVectors.get(i).getMagnitude() / spacing)));
	   }
//	   for(int i = 0; i < newVectors.size(); i++) {
//		  System.out.println(i + " Number of Points in Line Segment:" + newNumOPoints.get(i).intValue());
//	   }
	   for(int i = 0; i < newVectors.size(); i++) {
		      double magnitude = newVectors.get(i).getMagnitude();
			  newVectors.get(i).setX((newVectors.get(i).getX() / magnitude) * spacing);
			  newVectors.get(i).setY((newVectors.get(i).getY() / magnitude) * spacing);
	   }
//	   for(int i = 0; i < newVectors.size(); i++) {
//		      System.out.println(newVectors.get(i).getX());
//		      System.out.println(newVectors.get(i).getY());
//	   }
	   for (int i = 0; i < newNumOPoints.size(); i++) {
		   for(int j = 0; j < newNumOPoints.get(i); j++) {
			   finalPoints.add(new Point3(newPoints.get(i).getX() + newVectors.get(i).getX() * j, 
					           newPoints.get(i).getY() + newVectors.get(i).getY() * j));
		   }
		   
	   }
	   finalPoints.add(newPoints.get(newPoints.size()-1));
//	   for(int i = 0; i < finalPoints.size(); i++) {
////		   System.out.println(i + " X:" + finalPoints.get(i).getX());
//		   System.out.println(i + " Y:" + finalPoints.get(i).getY());
//	   }
   }
}