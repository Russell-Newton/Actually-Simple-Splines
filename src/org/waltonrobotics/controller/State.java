package org.waltonrobotics.controller;

/**
 * Holds an encoder length, velocity, and acceleration
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class State {

	private final double length;
	private final double velocity;
	private final double acceleration;

	/**
	 * @param length
	 *            - the encoder length at the point
	 * @param velocity
	 *            - the velocity at the point
	 * @param acceleration
	 *            - the acceleration at the point
	 */
	public State(double length, double velocity, double acceleration) {
		this.length = length;
		this.velocity = velocity;
		this.acceleration = acceleration;
	}

	/**
	 * @return length - the encoder length at the point
	 */
	public double getLength() {
		return length;
	}

	/**
	 * @return velocity - the velocity at the point
	 */
	public double getVelocity() {
		return velocity;
	}

	/**
	 * @return acceleration - the acceleration at the point
	 */
	public double getAcceleration() {
		return acceleration;
	}

}
