package org.waltonrobotics.controller;

public class PathData {
	
	private final State[] leftStates;
	private final State[] rightStates;
	private final Pose[] centerPoses;
	private final double[] times;

	public PathData(State[] leftStates, State[] rightStates, Pose[] centerPoses, double times[]) {
		this.leftStates = leftStates;
		this.rightStates = rightStates;
		this.centerPoses = centerPoses;
		this.times = times;
	}
	
	public State[] getLeftStates() {
		return leftStates;
	}
	
	public State[] getRightStates() {
		return rightStates;
	}
	
	public Pose[] getCenterPoses() {
		return centerPoses;
	}
	
	public double[] getTimes() {
		return times;
	}
}
