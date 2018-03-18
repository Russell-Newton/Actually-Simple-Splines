package org.waltonrobotics.controller;

/**
 * Contains information about the robot's motion at a specific time
 *
 * @author Russell Newton, Walton Robotics
 */
public class MotionData {

	private final Pose actual;
	private final Pose target;
	private final ErrorVector error;
	private final RobotPair powers;
	private final int pathNumber;
	private final MotionState currentMotionState;

	/**
	 * @param powers (as a RobotPair)
	 */
	public MotionData(Pose actual, Pose target, ErrorVector error, RobotPair powers,
		int pathNumber,
		MotionState currentMotionState) {
		this.actual = actual;
		this.target = target;
		this.error = error;
		this.powers = powers;
		this.pathNumber = pathNumber;
		this.currentMotionState = currentMotionState;
	}

	/**
	 * @return actualPose
	 */
	public final Pose getActualPose() {
		return actual;
	}

	/**
	 * @return targetPose
	 */
	public final Pose getTargetPose() {
		return target;
	}

	/**
	 * @return error ErrorVector
	 */
	public final ErrorVector getError() {
		return error;
	}

	/**
	 * @return powers and time RobotPair
	 */
	public final RobotPair getPowers() {
		return powers;
	}

	public MotionState getCurrentMotionState() {
		return currentMotionState;
	}

	@Override
	public String toString() {
		return "MotionData{" +
			"actual=" + actual +
			", target=" + target +
			", error=" + error +
			", powers=" + powers +
			", pathNumber=" + pathNumber +
			", currentMotionState=" + currentMotionState +
			'}';
	}

	public int getPathNumber() {
		return pathNumber;
	}
}
