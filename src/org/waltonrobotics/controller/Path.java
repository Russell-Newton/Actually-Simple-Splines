package org.waltonrobotics.controller;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * Extend this if you want to make your own Motion.
 *
 * @author Russell Newton, Walton Robotics
 */
public abstract class Path {

	private static double robotWidth;
	private final double vCruise;
	private final double aMax;
	private final boolean isBackwards;
	private final List<Pose> keyPoints;
	private boolean isFinished;

	/**
	 * @param vCruise - cruise velocity
	 * @param aMax - max acceleration
	 */
	public Path(double vCruise, double aMax, boolean isBackwards, List<Pose> keyPoints) {
		this.isBackwards = isBackwards;
		this.keyPoints = keyPoints;
		if (vCruise == 0) {
			throw new IllegalArgumentException("vCruise cannot be 0");
		}
		this.vCruise = vCruise;

		if (aMax == 0) {
			throw new IllegalArgumentException("aMax cannot be 0");
		}
		this.aMax = aMax;
		isFinished = false;
	}

	public Path(double vCruise, double aMax, boolean isBackwards, Pose... keyPoints) {
		this(vCruise, aMax, isBackwards, Arrays.asList(keyPoints));
	}

	public static double getRobotWidth() {
		return robotWidth;
	}

	public static void setRobotWidth(double robotWidth) {
		Path.robotWidth = robotWidth;
	}

	public final boolean isBackwards() {
		return isBackwards;
	}

	public final List<Pose> getKeyPoints() {
		return keyPoints;
	}

	public final boolean isFinished() {
		return isFinished;
	}

	public final void setFinished(boolean finished) {
		isFinished = finished;
	}

	public final double getAMax() {
		return aMax;
	}

	/**
	 * @return the path data for the whole path
	 * @see PathData
	 */
	public abstract LinkedList<PathData> getPathData();

	@Override
	public String toString() {
		return "Path{" +
			"vCruise=" + vCruise +
			", aMax=" + aMax +
			", isFinished=" + isFinished +
			", robotWidth=" + robotWidth +
			'}';
	}

	public final double getVCruise() {
		return vCruise;
	}
}
