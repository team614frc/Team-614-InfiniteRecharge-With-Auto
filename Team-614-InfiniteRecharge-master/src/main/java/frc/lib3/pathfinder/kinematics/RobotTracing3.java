package frc.lib3.pathfinder.kinematics;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Optional;

import frc.lib3.util.Point3;

public class RobotTracing3 {

	public ArrayList<Point3> path = new ArrayList<Point3>();
	public ArrayList<Point3> leftPath = new ArrayList<Point3>();
	public ArrayList<Point3> rightPath = new ArrayList<Point3>();
	public ArrayList<Double> heading = new ArrayList<Double>();
	public double robotWidth;

	public RobotTracing3(ArrayList<Point3> path, double trackWidth) {
		this.path = path;
		this.robotWidth = trackWidth;
	}

	public void leftRight(ArrayList<Point3> path, double trackWidth) {
		ArrayList<Double> gradient = new ArrayList<Double>();

		for (int i = 0; i < path.size() - 1; i++)
			gradient.add(Math.atan2(path.get(i + 1).getY() - path.get(i).getY(),
					path.get(i + 1).getX() - path.get(i).getX()));

		gradient.set(gradient.size() - 1, gradient.get(gradient.size() - 2));

		for (int i = 0; i < gradient.size(); i++) {
			leftPath.add(new Point3(
					(((trackWidth / 2) * Math.cos(gradient.get(i) + Math.PI / 2)) + path.get(i).getX()),
					(((trackWidth / 2) * Math.sin(gradient.get(i) + Math.PI / 2)) + path.get(i).getY())));

			rightPath.add(new Point3(
					(((trackWidth / 2) * Math.cos(gradient.get(i) - Math.PI / 2)) + path.get(i).getX()),
					(((trackWidth / 2) * Math.sin(gradient.get(i) - Math.PI / 2)) + path.get(i).getY())));
			
			
			double deg = Math.toDegrees(gradient.get(i));
			gradient.set(i, deg);
			if (i > 0) {
				if ((deg - gradient.get(i - 1)) > 180)
					gradient.set(i, -360 + deg);
				if ((deg - gradient.get(i - 1)) < -180)
					gradient.set(i, 360 + deg);
			}
		}
		this.heading = gradient;
	}
}
