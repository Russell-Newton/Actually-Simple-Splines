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

	/**
	 * @param left
	 *            - the left encoder distance
	 * @param right
	 *            - the right encoder distance
	 */
	public RobotPair(double left, double right) {
		this.left = left;
		this.right = right;
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

}
