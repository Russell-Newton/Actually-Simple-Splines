package org.waltonrobotics.controller;

/**
 * This holds the x, y, and angle of the robot.
 *
 * @author Russell Newton, Walton Robotics
 */
public class Pose {

	private final double x;
	private final double y;
	private final double angle;

	/**
	 *
	 * @param x
	 * @param y
	 * @param angle
	 */
	public Pose(double x, double y, double angle) {
		this.x = x;
		this.y = y;
		this.angle = angle;
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
	public final double getX() {
		return x;
	}

	/**
	 * @return the y value of the point
	 */
	public final double getY() {
		return y;
	}

	/**
	 * @return the derivative of the point
	 */
	public final double getAngle() {
		return angle;
	}

	/**
	 * Finds the distance between two points
	 *
	 * @param otherPoint - the point you want to find the distance from
	 * @return the distance from this point to the other point
	 */
	public final double distance(Pose otherPoint) {
		if (getX() == otherPoint.getX() && getY() == otherPoint.getY()) {
			return Math.abs(otherPoint.getAngle() - getAngle()); //TODO check this
		} else {
			return Math.sqrt(
				StrictMath.pow(x - otherPoint.x, 2) + StrictMath
					.pow(y - otherPoint.y, 2));
		}
	}

	/**
	 * Rotates a point around a central point. Imagine making an arc on a circle
	 *
	 * @param centerPoint - the center of the circle
	 * @param arcAngle - the angle to rotate the point to (degrees)
	 * @param backwards - whether or not to rotate the point backwards (clockwise)
	 * @return the rotated point
	 */
	public final Pose rotate(Pose centerPoint, double arcAngle, boolean backwards, double scale) {
		double distance = distance(centerPoint) * (backwards ? -1 : 1) * scale;
		double yDisplacement = distance * StrictMath.sin(arcAngle);
		double xDisplacement = distance * StrictMath.cos(arcAngle);
		return new Pose(centerPoint.x + xDisplacement, centerPoint.y + yDisplacement);
	}

	public final boolean sameCoordinates(Pose other)
	{
		return other.getX() == getX() && other.getY() == getY();
	}

	/**
	 * Creates an offset Pose by a dX, dY, and dAngle
	 *
	 * @return the offset Pose
	 */
	public final Pose offset(double dX, double dY, double dAngle) {
		return new Pose(x + dX, y + dY, angle + dAngle);
	}

	@Override
	public final String toString() {
		return "Pose{" +
			"x=" + x +
			", y=" + y +
			", angle=" + angle +
			'}';
	}
}
