package org.waltonrobotics.controller;

/**
 * This class can be created with various attributes. If you want to design your
 * own extended path motions, you'll probably want to use the bulky constructor.
 * If you just want to make one of ours, then you should just use the
 * constructor with an x and y
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class Pose {

	private final double x;
	private final double y;
	private final double angle;

	/**
	 * Can be used to create a point with just x, y, and derivative
	 * 
	 * @param x
	 *            - the x at the Pose
	 * @param y
	 *            - the y at the Pose
	 * @param angle
	 *            - the angle at the Pose
	 */
	public Pose(double x, double y, double angle) {
		this.x = x;
		this.y = y;
		this.angle = angle;
	}
	
	public Pose(double x, double y) {
		this(x, y, 0);
	}

	/**
	 * @return the x value of the point
	 */
	public double getX() {
		return x;
	}

	/**
	 * @return the y value of the point
	 */
	public double getY() {
		return y;
	}

	/**
	 * @return the derivative of the point
	 */
	public double getAngle() {
		return angle;
	}

	/**
	 * Finds the distance between two points
	 * 
	 * @param otherPoint
	 *            - the point you want to find the distance from
	 * @return the distance from this point to the other point
	 */
	public double distance(Pose otherPoint) {
		return Math.sqrt(Math.pow(this.x - otherPoint.getX(), 2) + Math.pow(this.y - otherPoint.getY(), 2));
	}

	/**
	 * Rotates a point around a central point. Imagine making an arc on a circle
	 * 
	 * @param centerPoint
	 *            - the center of the circle
	 * @param arcAngle
	 *            - the angle to rotate the point to (degrees)
	 * @param backwards
	 *            - whether or not to rotate the point backwards
	 * @return the rotated point
	 */
	public Pose rotate(Pose centerPoint, double arcAngle, boolean backwards) {
		double distance = this.distance(centerPoint) * (backwards ? -1 : 1);
		double y_displacement = distance * Math.sin(Math.toRadians(arcAngle));
		double x_displacement = distance * Math.cos(Math.toRadians(arcAngle));
		return new Pose(centerPoint.getX() + x_displacement, centerPoint.getY() + y_displacement);
	}
	
	public Pose offset(double dX, double dY, double dAngle) {
		return new Pose(this.x + dX, this.y + dY, this.angle + dAngle);
	}

}
