package org.waltonrobotics.controller;

public class PathData {

	private final State leftState;
	private final State rightState;
	private final Pose centerPose;
	private final double time;

	public PathData(State leftState, State rightState, Pose centerPose, double time) {
		this.leftState = leftState;
		this.rightState = rightState;
		this.centerPose = centerPose;
		this.time = time;
	}

	public PathData(Pose centerPose) {
		this(new State(0, 0, 0), new State(0, 0, 0), centerPose, 0);
	}

	public State getLeftState() {
		return leftState;
	}

	public State getRightState() {
		return rightState;
	}

	public Pose getCenterPose() {
		return centerPose;
	}

	public double getTime() {
		return time;
	}

	public double getLCenter() {
		return (leftState.getLength() + rightState.getLength()) / 2;
	}
}
