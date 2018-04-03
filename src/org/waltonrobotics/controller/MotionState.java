package org.waltonrobotics.controller;

/**
 * The state that the robot is in right now.
 */
enum MotionState {
	/**
	 * The robot is moving through the path motion
	 */
	MOVING,
	/**
	 * The robot has finished moving through the path no it is integrating the residual errors in order to finish in the
	 * correct spot
	 */
	FINISHING,
	/**
	 * The robot is waiting for a motion to be added. It will not move.
	 */
	WAITING
}