package org.waltonrobotics.motion;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;

/**
 * Extend this if you want to make your own Motion.
 *
 * @author Russell Newton, Walton Robotics
 */
public abstract class Path {

	//FIXME 1000 points per meter?
	public static int pathNumberOfSteps = 1000; // TODO find better name for this variable. Also before it was 50 but maybe try smart
	private static double robotWidth; // WHat if you have multiple robots running the same code? Should we account for that scenario?
	private final double vCruise;
	private final double aMax;
	private final boolean isBackwards;
	private final List<Pose> keyPoints;
	private final LinkedList<PathData> pathData;
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
		pathData = new LinkedList<>();
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
		} else if (angle < -Math.PI) {
			return angle + (2 * Math.PI);
		}
		return angle;
	}

	/**
	 * @return the path data for the whole path
	 * @see PathData
	 */
	public LinkedList<PathData> getPathData() {
		return pathData;
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

	public void setKeyPoints(Collection<Pose> keyPoints) {

		this.keyPoints.clear();
		this.keyPoints.addAll(keyPoints);

		createPath();
	}

	public void setKeyPoints(Pose... keyPoints) {
		this.keyPoints.clear();
		Collections.addAll(this.keyPoints, keyPoints);

		createPath();
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
	 * @return the velocity the robot should try to reach
	 */
	public final double getVCruise() {
		return vCruise;
	}

	public abstract void createPath();

	@Override
	public String toString() {
		return "Path{" +
			"vCruise=" + vCruise +
			", aMax=" + aMax +
			", isBackwards=" + isBackwards +
			", keyPoints=" + keyPoints +
			", pathData=" + pathData +
			", isFinished=" + isFinished +
			'}';
	}
}
