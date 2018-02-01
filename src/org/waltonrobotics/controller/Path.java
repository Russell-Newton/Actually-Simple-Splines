package org.waltonrobotics.controller;

/**
 * Defines the methods used by a motion path
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public abstract class Path {

	protected double vCruise;
	protected double aMax;
	private RobotPair startingWheelPositions = new RobotPair(0, 0);
	public boolean isFinished = false;

	/**
	 * @param vCruise
	 *            - cruise velocity
	 * @param aMax
	 *            - max acceleration
	 */
	protected Path(double vCruise, double aMax) {
		if (vCruise == 0)
			throw new IllegalArgumentException("vCruise cannot be 0");
		this.vCruise = vCruise;

		if (aMax == 0)
			throw new IllegalArgumentException("aMax cannot be 0");
		this.aMax = aMax;
	}

	/**
	 * This method should return an array of points defining the center of the path
	 */
	public abstract Point[] getPathPoints();

	/**
	 * This method should return an array of points defining the left side of a path
	 */
	public abstract Point[] getLeftPath();

	/**
	 * This method should return an array of points defining the right side of a
	 * path
	 */
	public abstract Point[] getRightPath();

	/**
	 * This will find the encoder lengths to use for the calculations in
	 * MotionController
	 * 
	 * @param currentTime
	 *            - the time since the path has started running
	 * @return - a State array that holds the left and right states
	 */
	public State[] interpolatePosition(double currentTime) {
		int index = 0;

		// iterate through the path until you find the points you are between
		while (currentTime >= getLeftPath()[index].getTime()) {
			index++;
		}
		index--;
		Point leftPrevious = getLeftPath()[index];
		Point rightPrevious = getRightPath()[index];
		// if you're at or past the last point, return the last point set's state
		if (index >= getLeftPath().length - 2) {
			isFinished = true;
			return new State[] {
					new State(leftPrevious.getLength() + startingWheelPositions.getLeft(), leftPrevious.getVelocity(),
							leftPrevious.getAcceleration()),
					new State(rightPrevious.getLength() + startingWheelPositions.getRight(),
							rightPrevious.getVelocity(), rightPrevious.getAcceleration()) };
		}

		Point leftNext = getLeftPath()[index + 1];
		Point rightNext = getRightPath()[index + 1];

		double dTime = leftNext.getTime() - leftPrevious.getTime();

		double rctn = (leftNext.getTime() - currentTime) / dTime; // Ratio of the current time to the next point time
		double rltc = (currentTime - leftPrevious.getTime()) / dTime; // Ratio of the previous time to the current point
																		// time

		// New lengths are the weighted averages of the point lengths
		double lengthLeft = (leftPrevious.getLength() + startingWheelPositions.getLeft()) * rctn
				+ (leftNext.getLength() + startingWheelPositions.getLeft()) * rltc;
		double lengthRight = (rightPrevious.getLength() + startingWheelPositions.getRight()) * rctn
				+ (rightNext.getLength() + startingWheelPositions.getRight()) * rltc;

		System.out.println(
				"Current index: " + index + "\t Current time: " + currentTime + "\t Next time: " + leftNext.getTime());
		System.out.println("Next Left: " + leftNext.getLength() + " Next Right: " + rightNext.getLength());

		return new State[] { new State(lengthLeft, leftNext.getVelocity(), leftNext.getAcceleration()),
				new State(lengthRight, rightNext.getVelocity(), rightNext.getAcceleration()) };
	}

	/**
	 * This is used to set define the wheel positions when the path is started. This
	 * lets the paths themselves use relative lengths while not having to reset the
	 * encoders
	 * 
	 * @param positions
	 *            - the RobotPair with the wheel positions
	 */
	public void setStartingWheelPositions(RobotPair positions) {
		startingWheelPositions = positions;
	}
}
