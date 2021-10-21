package frc.lib2.util;

public class Vector2 {
   private double x;
   private double y;

   public Vector2(double x, double y) {
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
   
   public Vector2 normalizeVector() {
	  return null;   
   }
   
   public double dotProduct(Vector2 d) {
	  return this.getX() * d.getX() + this.getY() * d.getY();	   
   }
}
