package org.waltonrobotics.command;

import edu.wpi.first.wpilibj.command.Command;
import java.util.LinkedList;
import java.util.List;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;

public class LearnPath extends Command {

	private final int useLast;
	private final String fileName;
	private final List<PathData> moments;

	private double movementTolerance = .1;
	private double angleTolerance = 15;

	private double maxVelocity;
	private double maxAcceleration;

	public LearnPath(AbstractDrivetrain drivetrain, int useLast, String fileName) {
		this.useLast = useLast;
		this.fileName = fileName;
		moments = new LinkedList<>();
	}

	public LearnPath(AbstractDrivetrain drivetrain, int useLast) {
		this.useLast = useLast;
		moments = new LinkedList<>();
		fileName = null;
	}

	@Override
	protected void initialize() {
		moments.clear();
	}

	@Override
	protected void execute() {
		PathData instance = SimpleMotion.getDrivetrain().getCurrentRobotState();

		maxVelocity = Math.max(
			Math.max(
				Math.abs(instance.getLeftState().getVelocity()),
				Math.abs(instance.getRightState().getVelocity())
			),
			maxVelocity);

		maxAcceleration = Math.max(
			Math.max(
				Math.abs(instance.getLeftState().getAcceleration()),
				Math.abs(instance.getRightState().getAcceleration())
			),
			maxAcceleration);

		moments.add(instance);
	}


	@Override
	protected boolean isFinished() {

		if (moments.size() >= useLast) {

			double movementChange = 0;
			double angleChange = 0;

			int size = moments.size();

			for (int i = size - (useLast - 1) - 1; i < size; i++) {
				Pose currentPosition = moments.get(i).getCenterPose();
				Pose previousPosition = moments.get(i - 1).getCenterPose();

				double dX = Math.abs(currentPosition.getX() - previousPosition.getX());
				double dY = Math.abs(currentPosition.getY() - previousPosition.getY());
				double dAngle = Math.abs(currentPosition.getAngle() - previousPosition.getAngle());

				movementChange += dX + dY;
				angleChange += dAngle;
			}

			// If there is less than or equal to a 10 cm change and there is less than or equal to 15 degrees of change
			return (movementChange <= movementTolerance) &&
				(angleChange <= StrictMath.toRadians(angleTolerance));
		}
		return false;
	}

	public List<PathData> getMoments() {
		return moments;
	}

	public double getMaxVelocity() {
		return maxVelocity;
	}

	public double getMaxAcceleration() {
		return maxAcceleration;
	}

	public String getFileName() {
		return fileName;
	}

	public int getUseLast() {
		return useLast;
	}

	public double getMovementTolerance() {
		return movementTolerance;
	}

	public void setMovementTolerance(double movementTolerance) {
		this.movementTolerance = movementTolerance;
	}

	public double getAngleTolerance() {
		return angleTolerance;
	}

	public void setAngleTolerance(double angleTolerance) {
		this.angleTolerance = angleTolerance;
	}

	@Override
	public String toString() {
		return "LearnPath{" +
			"useLast=" + useLast +
			", fileName='" + fileName + '\'' +
			", moments=" + moments +
			'}';
	}
}
