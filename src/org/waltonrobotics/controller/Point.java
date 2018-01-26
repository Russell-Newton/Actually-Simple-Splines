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
public class Point {

	private final double x;
	private final double y;
	private final double derivative;
	private final State state;
	private final double lCenter;
	private final double time;

	/**
	 * Used to create a point
	 * 
	 * @param x
	 *            - the x at the point
	 * @param y
	 *            - the y at the point
	 * @param derivative
	 *            - the derivative at the point, used for offsetting points
	 * @param state
	 *            - a State with the encoder length, velocity, and acceleration at
	 *            the point
	 * @param lCenter
	 *            - average desired encoder distance
	 * @param time
	 *            - the expected time to reach the point
	 */
	public Point(double x, double y, double derivative, State state, double lCenter, double time) {
		this.x = x;
		this.y = y;
		this.derivative = derivative;
		this.state = state;
		this.lCenter = lCenter;
		this.time = time;
	}

	/**
	 * Can be used to create a point with just x, y, and derivative
	 * 
	 * @param x
	 *            - the x at the point
	 * @param y
	 *            - the y at the point
	 * @param derivative
	 *            - the derivative at the point, used for offsetting points
	 */
	public Point(double x, double y, double derivative) {
		this(x, y, derivative, new State(0, 0, 0), 0, 0);
	}

	/**
	 * Can be used to create a point with just x and y
	 * 
	 * @param x
	 *            - the x at the point
	 * @param y
	 *            - the y at the point
	 */
	public Point(double x, double y) {
		this(x, y, 0, new State(0, 0, 0), 0, 0);
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
	public double getDerivative() {
		return derivative;
	}

	/**
	 * @return the velocity at the point
	 */
	public double getVelocity() {
		return state.getVelocity();
	}

	/**
	 * @return the acceleration at the point
	 */
	public double getAcceleration() {
		return state.getAcceleration();
	}

	/**
	 * @return encoder length at the point
	 */
	public double getLength() {
		return state.getLength();
	}

	/**
	 * @return average encoder length at the point
	 */
	public double getLCenter() {
		return lCenter;
	}

	/**
	 * @return time at the point
	 */
	public double getTime() {
		return time;
	}

	/**
	 * Offsets a point along a perpendicular line from a tangent line
	 * 
	 * @param dtAtPoint
	 *            - the derivative of the point
	 * @param distance
	 *            - the distance to offset the point by
	 * @param state
	 *            - a State with the encoder length, velocity, and acceleration at
	 *            the point
	 * @param lCenter
	 *            - average desired encoder distance
	 * @param time
	 *            - the expected time to reach the point
	 * @return the offset point
	 */
	public Point offsetPerpendicular(double distance, State state, double lCenter, double time) {
		double angleOfDT = Math.atan(this.derivative);
		double offsetX = distance * Math.cos(angleOfDT + Math.PI / 2); // Finds point at distance along perpendicular
																		// line
		double offsetY = distance * Math.sin(angleOfDT + Math.PI / 2);

		return new Point(this.x + offsetX, this.y + offsetY, angleOfDT, state, lCenter, time);
	}

	/**
	 * Finds the distance between two points
	 * 
	 * @param otherPoint
	 *            - the point you want to find the distance from
	 * @return the distance from this point to the other point
	 */
	public double distance(Point otherPoint) {
		return Math.sqrt(Math.pow(this.x - otherPoint.getX(), 2) + Math.pow(this.y - otherPoint.getY(), 2));
	}

	/**
	 * Rotates a point around a central point. Imagine making an arc on a circle
	 * 
	 * @param centerPoint
	 *            - the center of the circle
	 * @param arcAngle
	 *            - the angle to rotate the point to (degrees)
	 * @return the rotated point
	 */
	public Point rotate(Point centerPoint, double arcAngle) {
		double distance = this.distance(centerPoint);
		double y_displacement = distance * Math.sin(Math.toRadians(arcAngle));
		double x_displacement = distance * Math.cos(Math.toRadians(arcAngle));
		return new Point(centerPoint.getX() + x_displacement, centerPoint.getY() + y_displacement);
	}

}
