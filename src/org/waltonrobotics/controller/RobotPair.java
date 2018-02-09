package org.waltonrobotics.controller;

/**
 * Utilized to identify left and right encoder measurements
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class RobotPair {

	private final double left;
	private final double right;
	private final double time;

	/**
	 * @param left
	 *            - the left encoder distance
	 * @param right
	 *            - the right encoder distance
	 */
	public RobotPair(double left, double right, double time) {
		this.left = left;
		this.right = right;
		this.time = time;
	}

	/**
	 * @return - the left encoder distance
	 */
	public double getLeft() {
		return left;
	}

	/**
	 * @return - the right encoder distance
	 */
	public double getRight() {
		return right;
	}
	
	/**
	 * @return - time when the encoders were measured
	 */
	public double getTime() {
		return time;
	}

}
