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
	 * @param vCruise cruise velocity of the robot, the velocity that the robot should try to reach
	 * @param aMax the maximum acceleration the robot should achieve
	 * @param isBackwards if the robot is travelling forwards or backwards
	 * @param keyPoints the points that define the path
	 */
	protected Path(double vCruise, double aMax, boolean isBackwards, List<Pose> keyPoints) {
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

	/**
	 * @return the number of steps the path should be divided into. Default is 50.
	 */
	public static int getPathNumberOfSteps() {
		return pathNumberOfSteps;
	}

	/**
	 * @param pathNumberOfSteps the new number of steps a path should be divided into
	 */
	public static void setPathNumberOfSteps(int pathNumberOfSteps) {
		Path.pathNumberOfSteps = pathNumberOfSteps;
	}

	/**
	 * @return The width of the robot from the outside of each wheel
	 */
	public static double getRobotWidth() {
		return robotWidth;
	}

	/**
	 * @param robotWidth the new width of the robot
	 */
	public static void setRobotWidth(double robotWidth) {
		Path.robotWidth = robotWidth;
	}

	/**
	 * Bounds an angle to be in between -PI and PI. if the angles are more or less then the angle will cycle.
	 *
	 * @param angle angle to be bounded
	 * @return the angle bounded
	 */
	public static double boundAngle(double angle) {
		if (angle > Math.PI) {
			return angle - (2.0 * Math.PI);
		}
		if (angle < -Math.PI) {
			return angle + (2 * Math.PI);
		}
		return angle;
	}

	/**
	 * @return if the robot is travelling backwards or not
	 */
	public final boolean isBackwards() {
		return isBackwards;
	}

	/**
	 * @return the key points that define the path
	 */
	public final List<Pose> getKeyPoints() {
		return keyPoints;
	}

	/**
	 * @return if the path has been completed
	 */
	public final boolean isFinished() {
		return isFinished;
	}

	public final void setFinished(boolean finished) {
		isFinished = finished;
	}

	/**
	 * @return the maximum acceleration the robot should be at
	 */
	public final double getAMax() {
		return aMax;
	}

	/**
	 * @return the path data for the whole path
	 * @see PathData
	 */
	public abstract LinkedList<PathData> getPathData();

	/**
	 * @return the velocity the robot should try to reach
	 */
	public final double getVCruise() {
		return vCruise;
	}

	@Override
	public String toString() {
		return "Path{" +
			"vCruise=" + vCruise +
			", aMax=" + aMax +
			", isBackwards=" + isBackwards +
			", keyPoints=" + keyPoints +
			", isFinished=" + isFinished +
			'}';
	}
}
