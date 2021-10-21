package frc.lib3.util;

public class Vector3 {
   private double x;
   private double y;

   public Vector3(double x, double y) {
	   this.x = x;
	   this.y = y;   
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
   
   public double getMagnitude() {
	   return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
   }
   
   public Vector3 normalizeVector() {
	  return null;   
   }
   
   public double dotProduct(Vector3 d) {
	  return this.getX() * d.getX() + this.getY() * d.getY();	   
   }
}
