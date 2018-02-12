package org.waltonrobotics.controller;

/**
 * Contains information about the robot's motion at a specific time
 * @author Russell Newton, Walton Robotics
 *
 */
public class MotionData {
	private final Pose actual;
	private final Pose target;
	private final ErrorVector error;
	private final RobotPair powers;
	
	/**
	 * 
	 * @param actual
	 * @param target
	 * @param error
	 * @param powers (as a RobotPair)
	 */
	public MotionData(Pose actual, Pose target, ErrorVector error, RobotPair powers) {
		this.actual = actual;
		this.target = target;
		this.error = error;
		this.powers = powers;
	}
	
	/**
	 * 
	 * @return actualPose
	 */
	public Pose getActualPose() {
		return actual;
	}
	
	/**
	 * 
	 * @return targetPose
	 */
	public Pose getTargetPose() {
		return target;
	}
	
	/**
	 * 
	 * @return error ErrorVector
	 */
	public ErrorVector getError() {
		return error;
	}
	
	/**
	 * 
	 * @return powers and time RobotPair
	 */
	public RobotPair getPowers() {
		return powers;
	}
}
