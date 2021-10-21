package frc.lib3.util;

public class Point3 {
   private double x;
   private double y;

   public Point3(double x, double y) {
	   this.x = x;
	   this.y = y;   
   }
   
	// TODO Auto-generated constructor stub

   public Point3(Point3 point) {
	   this.x = point.getX();
	   this.y = point.getY();
   }
	// TODO Auto-generated constructor stub


   public double getX() {
	   return x;
   }
   
   public double getY() {
	   return y;
   }
   
   public void setY(double y) {
	   this.y = y;
   }
   
   public void setX(double x) {
	   this.x = x;
   }
   
   public double distance(Point3 x) {
	   return Math.sqrt(Math.pow( x.getX() - this.getX() , 2) + Math.pow( x.getY() - this.getY() , 2));
   }
}
