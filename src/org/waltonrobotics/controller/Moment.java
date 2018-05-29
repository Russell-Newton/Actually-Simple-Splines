package org.waltonrobotics.controller;

public class Moment {

	private final Pose actualPosition;
	private final RobotPair wheelPosition;
	public Moment(double x, double y, double angle, double leftLength, double rightLength, double time) {
		this(new Pose(x, y, angle), new RobotPair(leftLength, rightLength, time));
	}

	public Moment(Pose actualPosition, RobotPair wheelPosition) {

		this.actualPosition = actualPosition;
		this.wheelPosition = wheelPosition;
	}

	@Override
	public String toString() {
		return "Moment{" +
			"actualPosition=" + actualPosition +
			", wheelPosition=" + wheelPosition +
			'}';
	}

	public Pose getActualPosition() {
		return actualPosition;
	}

	public RobotPair getWheelPosition() {
		return wheelPosition;
	}

	public double getX() {
		return actualPosition.getX();
	}

	public double getY() {
		return actualPosition.getY();
	}

	public double getAngle() {
		return actualPosition.getAngle();
	}

	public double getLeftLength() {
		return wheelPosition.getLeft();
	}

	public double getRightLength() {
		return wheelPosition.getRight();
	}

	public double getTime() {
		return wheelPosition.getTime();
	}
}
