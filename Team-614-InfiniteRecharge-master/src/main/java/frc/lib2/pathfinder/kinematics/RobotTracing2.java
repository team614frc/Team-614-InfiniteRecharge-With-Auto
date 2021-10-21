package frc.lib2.pathfinder.kinematics;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Optional;

import frc.lib2.util.Point2;

public class RobotTracing2 {

	public ArrayList<Point2> path = new ArrayList<Point2>();
	public ArrayList<Point2> leftPath = new ArrayList<Point2>();
	public ArrayList<Point2> rightPath = new ArrayList<Point2>();
	public ArrayList<Double> heading = new ArrayList<Double>();
	public double robotWidth;

	public RobotTracing2(ArrayList<Point2> path, double trackWidth) {
		this.path = path;
		this.robotWidth = trackWidth;
	}

	public void leftRight(ArrayList<Point2> path, double trackWidth) {
		ArrayList<Double> gradient = new ArrayList<Double>();

		for (int i = 0; i < path.size() - 1; i++)
			gradient.add(Math.atan2(path.get(i + 1).getY() - path.get(i).getY(),
					path.get(i + 1).getX() - path.get(i).getX()));

		gradient.set(gradient.size() - 1, gradient.get(gradient.size() - 2));

		for (int i = 0; i < gradient.size(); i++) {
			leftPath.add(new Point2(
					(((trackWidth / 2) * Math.cos(gradient.get(i) + Math.PI / 2)) + path.get(i).getX()),
					(((trackWidth / 2) * Math.sin(gradient.get(i) + Math.PI / 2)) + path.get(i).getY())));

			rightPath.add(new Point2(
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
