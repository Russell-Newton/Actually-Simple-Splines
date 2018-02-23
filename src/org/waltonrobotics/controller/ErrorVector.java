package org.waltonrobotics.controller;

/**
 * Holds the cross-track and lag errors
 *
 * @author Russell Newton, Walton Robotics
 */
public class ErrorVector {

	private final double lag;
	private final double crossTrack;
	private final double angle;

	/**
	 * @param lag
	 * @param crossTrack
	 */
	public ErrorVector(double lag, double crossTrack, double angle) {
		this.lag = lag;
		this.crossTrack = crossTrack;
		this.angle = angle;
	}

	/**
	 * @return lag error
	 */
	public double getLag() {
		return lag;
	}

	/**
	 * @return xtrack error
	 */
	public double getXTrack() {
		return crossTrack;
	}

	/**
	 * @return angle error
	 */
	public double getAngle() {
		return angle;
	}


	@Override
	public String toString() {
		return "ErrorVector{" +
			"lag=" + lag +
			", crossTrack=" + crossTrack +
			", angle=" + angle +
			'}';
	}
}
