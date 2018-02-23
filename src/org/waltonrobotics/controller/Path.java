package org.waltonrobotics.controller;

import java.util.Collections;
import java.util.List;

/**
 * Extend this if you want to make your own Motion.
 *
 * @author Russell Newton, Walton Robotics
 */
public abstract class Path {

	public final double robotWidth;
	private final boolean isBackwards;
	private final int numberOfSteps;
	private final List<Pose> keyPoints;
	public boolean isFinished;
	protected double vCruise;
	protected double aMax;

	/**
	 * @param vCruise - cruise velocity
	 * @param aMax - max acceleration
	 */
	protected Path(double vCruise, double aMax, double robotWidth, boolean isBackwards,
		int numberOfSteps, List<Pose> keyPoints) {
		this.isBackwards = isBackwards;
		this.numberOfSteps = numberOfSteps;
		this.keyPoints = keyPoints;

		if (vCruise == 0) {
			throw new IllegalArgumentException("vCruise cannot be 0");
		}
		this.vCruise = vCruise;

		if (aMax == 0) {
			throw new IllegalArgumentException("aMax cannot be 0");
		}
		this.aMax = aMax;
		this.robotWidth = robotWidth;
		isFinished = false;
	}

	public double getRobotWidth() {
		return robotWidth;
	}

	public boolean isBackwards() {
		return isBackwards;
	}

	public int getNumberOfSteps() {
		return numberOfSteps;
	}

	public List<Pose> getKeyPoints() {
		return keyPoints;
	}

	public boolean isFinished() {
		return isFinished;
	}

	public double getvCruise() {
		return vCruise;
	}

	public double getaMax() {
		return aMax;
	}

	/**
	 * @return the path data for the whole path
	 * @see PathData
	 */
	public abstract List<PathData> getPathData();

	public void addPoints(Pose... poses) {
		Collections.addAll(getKeyPoints(), poses);
		makePathData();
	}

	protected abstract void makePathData();

	@Override
	public String toString() {
		return "Path{" +
			"vCruise=" + vCruise +
			", aMax=" + aMax +
			", isFinished=" + isFinished +
			", robotWidth=" + robotWidth +
			'}';
	}
}
