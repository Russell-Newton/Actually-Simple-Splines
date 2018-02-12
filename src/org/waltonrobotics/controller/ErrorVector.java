package org.waltonrobotics.controller;

/**
 * Holds the cross-track and lag errors
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class ErrorVector {
	private final double lag;
	private final double crossTrack;

	/**
	 * @param lag
	 * @param crossTrack
	 */
	public ErrorVector(double lag, double crossTrack) {
		this.lag = lag;
		this.crossTrack = crossTrack;
	}

	/**
	 * 
	 * @return
	 */
	public double getLag() {
		return lag;
	}

	/**
	 * 
	 * @return
	 */
	public double getXTrack() {
		return crossTrack;
	}
}
