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

	public static int pathNumberOfSteps = 50; // TODO find better name for this variable
	private static double robotWidth; // WHat if you have multiple robots running the same code? Should we account for that scenario?
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

	public static int getPathNumberOfSteps() {
		return pathNumberOfSteps;
	}

	public static void setPathNumberOfSteps(int pathNumberOfSteps) {
		Path.pathNumberOfSteps = pathNumberOfSteps;
	}

	public static double getRobotWidth() {
		return robotWidth;
	}

	public static void setRobotWidth(double robotWidth) {
		Path.robotWidth = robotWidth;
	}

	public static double boundAngle(double angle) {
		if (angle > Math.PI) {
			return angle - 2 * Math.PI;
		}
		if (angle < -Math.PI) {
			return angle + 2 * Math.PI;
		}
		return angle;
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
