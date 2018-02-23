package org.waltonrobotics.controller;

/**
 * Holds information about the robot at a specific time in a Path
 *
 * @author Russell Newton, Walton Robotics
 */
public class PathData {

	private final State leftState;
	private final State rightState;
	private final Pose centerPose;
	private final double time;

	/**
	 * @param leftState
	 * @param rightState
	 * @param centerPose
	 * @param time
	 */
	public PathData(State leftState, State rightState, Pose centerPose, double time) {
		this.leftState = leftState;
		this.rightState = rightState;
		this.centerPose = centerPose;
		this.time = time;
	}

	/**
	 * Can be used to make a generic PathData without motion and time
	 */
	public PathData(Pose centerPose) {
		this(new State(0, 0, 0), new State(0, 0, 0), centerPose, 0);
	}

	/**
	 * @return leftState
	 */
	public State getLeftState() {
		return leftState;
	}

	/**
	 * @return rightState
	 */
	public State getRightState() {
		return rightState;
	}

	/**
	 * @return centerPose
	 */
	public Pose getCenterPose() {
		return centerPose;
	}

	/**
	 * @return time
	 */
	public double getTime() {
		return time;
	}

	/**
	 * @return the average length of the left and right States
	 */
	public double getLCenter() {
		return (leftState.getLength() + rightState.getLength()) / 2;
	}

	@Override
	public String toString() {
		return "PathData{" +
			"leftState=" + leftState +
			", rightState=" + rightState +
			", centerPose=" + centerPose +
			", time=" + time +
			'}';
	}
}
