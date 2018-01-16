package org.waltonrobotics.controller;

/**
 * Defines the methods used by a motion path
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public abstract class Path {

	protected double vCruise;
	protected double aMax;

	/**
	 * @param vCruise
	 *            - cruise velocity
	 * @param aMax
	 *            - max acceleration
	 */
	protected Path(double vCruise, double aMax) {
		if (vCruise == 0)
			throw new IllegalArgumentException("vCruise cannot be 0");
		this.vCruise = vCruise;

		if (aMax == 0)
			throw new IllegalArgumentException("aMax cannot be 0");
		this.aMax = aMax;
	}

	/**
	 * This method should return an array of points defining the center of the path
	 */
	public abstract Point[] getPathPoints();

	/**
	 * This method should return an array of points defining the left side of a path
	 */
	public abstract Point[] getLeftPath();

	/**
	 * This method should return an array of points defining the right side of a
	 * path
	 */
	public abstract Point[] getRightPath();
}
