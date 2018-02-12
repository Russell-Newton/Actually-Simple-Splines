package org.waltonrobotics.controller;

import java.util.List;

/**
 * Extend this if you want to make your own Motion.
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public abstract class Path {

	protected double vCruise;
	protected double aMax;
	public boolean isFinished;
	public final double robotWidth;

	/**
	 * @param vCruise
	 *            - cruise velocity
	 * @param aMax
	 *            - max acceleration
	 * @param robotWidth
	 */
	protected Path(double vCruise, double aMax, double robotWidth) {
		if (vCruise == 0)
			throw new IllegalArgumentException("vCruise cannot be 0");
		this.vCruise = vCruise;

		if (aMax == 0)
			throw new IllegalArgumentException("aMax cannot be 0");
		this.aMax = aMax;
		this.robotWidth = robotWidth;
		isFinished = false;
	}

	/**
	 * @return the path data for the whole path
	 * @see PathData
	 */
	public abstract List<PathData> getPathData();
}
