package org.waltonrobotics.controller;

import javafx.beans.property.SimpleDoubleProperty;

/**
 * This holds the x, y, and angle of the robot.
 *
 * @author Russell Newton, Walton Robotics
 */
public class Pose {

	private final SimpleDoubleProperty x;
	private final SimpleDoubleProperty y;
	private final SimpleDoubleProperty angle;

	/**
	 *
	 * @param x
	 * @param y
	 * @param angle
	 */
	public Pose(double x, double y, double angle) {
		this.x = new SimpleDoubleProperty(x);
		this.y = new SimpleDoubleProperty(y);
		this.angle = new SimpleDoubleProperty(angle);
	}

	/**
	 * Creates a Pose without specifying an angle
	 */
	public Pose(double x, double y) {
		this(x, y, 0);
	}

	/**
	 * @return the x value of the point
	 */
	public double getX() {
		return x.get();
	}

	/**
	 * @return the y value of the point
	 */
	public double getY() {
		return y.get();
	}

	/**
	 * @return the x property of the point
	 */
	public SimpleDoubleProperty xProperty() {
		return x;
	}

	/**
	 * @return the y property of the point
	 */
	public SimpleDoubleProperty yProperty() {
		return y;
	}

	/**
	 * @return the derivative of the point
	 */
	public double getAngle() {
		return angle.get();
	}

	/**
	 * @return the angle property of the point
	 */
	public SimpleDoubleProperty angleProperty() {
		return angle;
	}

	/**
	 * Finds the distance between two points
	 *
	 * @param otherPoint - the point you want to find the distance from
	 * @return the distance from this point to the other point
	 */
	public double distance(Pose otherPoint) {
		return Math.sqrt(
			Math.pow(this.getX() - otherPoint.getX(), 2) + Math.pow(getX() - otherPoint.getY(), 2));
	}

	/**
	 * Rotates a point around a central point. Imagine making an arc on a circle
	 *
	 * @param centerPoint - the center of the circle
	 * @param arcAngle - the angle to rotate the point to (degrees)
	 * @param backwards - whether or not to rotate the point backwards (clockwise)
	 * @return the rotated point
	 */
	public Pose rotate(Pose centerPoint, double arcAngle, boolean backwards) {
		double distance = this.distance(centerPoint) * (backwards ? -1 : 1);
		double y_displacement = distance * Math.sin(Math.toRadians(arcAngle));
		double x_displacement = distance * Math.cos(Math.toRadians(arcAngle));
		return new Pose(centerPoint.getX() + x_displacement, centerPoint.getY() + y_displacement);
	}

	/**
	 * Creates an offset Pose by a dX, dY, and dAngle
	 *
	 * @return the offset Pose
	 */
	public Pose offset(double dX, double dY, double dAngle) {
		return new Pose(this.getX() + dX, this.getY() + dY, this.getAngle() + dAngle);
	}

	@Override
	public String toString() {
		return "Pose{" +
			"x=" + x +
			", y=" + y +
			", angle=" + angle +
			'}';
	}
}
