package org.waltonrobotics.controller;

/**
 * Holds information about the left and right sides of the robot at a specific
 * time
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
	 * @param right
	 * @param time
	 */
	public RobotPair(double left, double right, double time) {
		this.left = left;
		this.right = right;
		this.time = time;
	}

	/**
	 * @return left
	 */
	public double getLeft() {
		return left;
	}

	/**
	 * @return right
	 */
	public double getRight() {
		return right;
	}

	/**
	 * @return time
	 */
	public double getTime() {
		return time;
	}

}
