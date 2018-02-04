package org.waltonrobotics.controller;

/**
 * Defines the methods used by a motion path
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public abstract class Path {

	// TODO - Make the getPathData method and all required

	protected double vCruise;
	protected double aMax;
	private RobotPair startingWheelPositions;
	public boolean isFinished = false;
	public final double robotWidth;

	/**
	 * @param vCruise
	 *            - cruise velocity
	 * @param aMax
	 *            - max acceleration
	 */
	protected Path(double vCruise, double aMax, double robotWidth) {
		if (vCruise == 0)
			throw new IllegalArgumentException("vCruise cannot be 0");
		this.vCruise = vCruise;

		if (aMax == 0)
			throw new IllegalArgumentException("aMax cannot be 0");
		this.aMax = aMax;
		this.robotWidth = robotWidth;
	}

	/**
	 * @return the path data for the whole path
	 * @see PathData
	 */
	public abstract PathData getPathData();

	/**
	 * This will find the encoder lengths to use for the calculations in
	 * MotionController
	 * 
	 * @param currentTime
	 *            - the time since the path has started running
	 * @return - a State array that holds the left and right states
	 */
	public State[] interpolateStates(double currentTime) {
		int index = 0;

		// iterate through the path until you find the points you are between
		while (currentTime >= getPathData().getTimes()[index]) {
			index++;
		}
		index--;
		State leftPrevious = getPathData().getLeftStates()[index];
		State rightPrevious = getPathData().getRightStates()[index];
		double timePrevious = getPathData().getTimes()[index];
		// if you're at or past the last point, return the last point set's state
		if (index >= getPathData().getCenterPoses().length - 2) {
			isFinished = true;
			return new State[] {
					new State(leftPrevious.getLength() + startingWheelPositions.getLeft(), leftPrevious.getVelocity(),
							leftPrevious.getAcceleration()),
					new State(rightPrevious.getLength() + startingWheelPositions.getRight(),
							rightPrevious.getVelocity(), rightPrevious.getAcceleration()) };
		}

		State leftNext = getPathData().getLeftStates()[index + 1];
		State rightNext = getPathData().getRightStates()[index + 1];
		double timeNext = getPathData().getTimes()[index + 1];

		double dTime = timeNext - timePrevious;

		double rctn = (timeNext - currentTime) / dTime; // Ratio of the current time to the next pose time
		double rltc = (currentTime - timePrevious) / dTime; // Ratio of the previous time to the current pose
															// time

		// New lengths are the weighted averages of the pose lengths
		double lengthLeft = (leftPrevious.getLength() + startingWheelPositions.getLeft()) * rctn
				+ (leftNext.getLength() + startingWheelPositions.getLeft()) * rltc;
		double lengthRight = (rightPrevious.getLength() + startingWheelPositions.getRight()) * rctn
				+ (rightNext.getLength() + startingWheelPositions.getRight()) * rltc;

		return new State[] { new State(lengthLeft, leftNext.getVelocity(), leftNext.getAcceleration()),
				new State(lengthRight, rightNext.getVelocity(), rightNext.getAcceleration()) };
	}

	public Pose updateTheoreticalPosition(double currentTime) {
		int index = 0;

		// iterate through the path until you find the points you are between
		while (currentTime >= getPathData().getTimes()[index]) {
			index++;
		}
		index--;
		Pose posePrevious = getPathData().getCenterPoses()[index];
		double timePrevious = getPathData().getTimes()[index];
		// if you're at or past the last point, return the last pose
		if (index >= getPathData().getCenterPoses().length - 2) {
			isFinished = true;
			return posePrevious;
		}
		Pose poseNext = getPathData().getCenterPoses()[index + 1];
		double timeNext = getPathData().getTimes()[index + 1];

		double dTime = timeNext - timePrevious;

		double rctn = (timeNext - currentTime) / dTime; // Ratio of the current time to the next pose time
		double rltc = (currentTime - timePrevious) / dTime; // Ratio of the previous time to the current pose
															// time
		// Current pose is made from the weighted average of the x, y, and angle values
		double x = posePrevious.getX() * rctn + poseNext.getX() * rltc;
		double y = posePrevious.getY() * rctn + poseNext.getY() * rltc;
		double angle = posePrevious.getAngle() * rctn + poseNext.getAngle() * rltc;
		return new Pose(x, y, angle);
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

	/**
	 * This is used in using the encoder lengths to find the actual position for the
	 * robot
	 * 
	 * @return - a Point with x, y, and angle
	 */
	public abstract Pose getStartingPosition();
}
