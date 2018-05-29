package org.waltonrobotics.command;

import edu.wpi.first.wpilibj.command.Command;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.controller.Moment;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.RobotPair;

public class LearnPath extends Command {

	private final int useLast;
	private final String fileName;
	//	private final List<Pose> robotPositions;
//	private final List<RobotPair> robotWheelPositions;
	private final List<Moment> moments;

	private double movementTolerance = .1;
	private double angleTolerance = 15;

	public LearnPath(AbstractDrivetrain drivetrain, int useLast, String fileName) {
		this.useLast = useLast;
		this.fileName = fileName;
//		robotPositions = new LinkedList<>();
//		robotWheelPositions = new LinkedList<>();
		moments = new LinkedList<>();
	}

	public LearnPath(AbstractDrivetrain drivetrain, int useLast) {
		this.useLast = useLast;
//		robotPositions = new LinkedList<>();
//		robotWheelPositions = new LinkedList<>();
		moments = new LinkedList<>();
		fileName = null;
	}

	public void setMovementTolerance(double movementTolerance) {
		this.movementTolerance = movementTolerance;
	}

	public void setAngleTolerance(double angleTolerance) {
		this.angleTolerance = angleTolerance;
	}

	@Override
	protected void initialize() {
//		robotPositions.clear();
//		robotWheelPositions.clear();
		moments.clear();
	}

	@Override
	protected void execute() {

		Pose actualPosition = SimpleMotion.getDrivetrain().getActualPosition();
		RobotPair wheelPosition = SimpleMotion.getDrivetrain().getWheelPositions();

//		robotWheelPositions.add(wheelPosition);
//		robotPositions.add(actualPosition);

		moments.add(new Moment(actualPosition, wheelPosition));
	}

	@Override
	protected void end() {
		if (fileName != null) {
			new Thread(this::savePoints).start();
		}
	}

	private void savePoints() {
		if (!moments.isEmpty() && (fileName != null)) {

			StringBuilder stringBuilder = new StringBuilder(
				moments.size() * 6 * 6);

			stringBuilder.append("X,Y,Angle,Left,Right,Time\n");

			Moment startingMoment = moments.get(0);

			for (Moment moment : moments) {
				stringBuilder.append(moment.getX() - startingMoment.getX());
				stringBuilder.append(',');
				stringBuilder.append(moment.getY() - startingMoment.getY());
				stringBuilder.append(',');
				stringBuilder.append(moment.getAngle() - startingMoment.getAngle());
				stringBuilder.append(',');
				stringBuilder.append(moment.getLeftLength() - startingMoment.getLeftLength());
				stringBuilder.append(',');
				stringBuilder.append(moment.getRightLength() - startingMoment.getRightLength());
				stringBuilder.append(',');
				stringBuilder.append(moment.getTime() - startingMoment.getTime());
				stringBuilder.append('\n');
			}

			try (BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(fileName))) {
				bufferedWriter.write(stringBuilder.toString());
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
//	private void savePoints() {
//		if (!robotWheelPositions.isEmpty() && fileName != null) {
//
//			StringBuilder stringBuilder = new StringBuilder(
//				robotPositions.size() * 3 * 3 + robotWheelPositions.size() * 3 * 3);
//
//			stringBuilder.append("X,Y,Angle,Left,Right,Time\n");
//
//			Pose startPosition = robotPositions.get(0);
//			RobotPair startWheelPosition = robotWheelPositions.get(0);
//
//			for (int i = 0; i < robotPositions.size(); i++) {
//				Pose position = robotPositions.get(i);
//				RobotPair wheelPosition = robotWheelPositions.get(i);
//
//				stringBuilder.append(position.getX() - startPosition.getX());
//				stringBuilder.append(',');
//				stringBuilder.append(position.getY() - startPosition.getY());
//				stringBuilder.append(',');
//				stringBuilder.append(position.getAngle() - startPosition.getAngle());
//				stringBuilder.append(',');
//				stringBuilder.append(wheelPosition.getLeft() - startWheelPosition.getLeft());
//				stringBuilder.append(',');
//				stringBuilder.append(wheelPosition.getRight() - startWheelPosition.getRight());
//				stringBuilder.append(',');
//				stringBuilder.append(wheelPosition.getTime() - startWheelPosition.getTime());
//				stringBuilder.append('\n');
//			}
//
//			try (BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(fileName))) {
//				bufferedWriter.write(stringBuilder.toString());
//			} catch (IOException e) {
//				e.printStackTrace();
//			}
//		}
//	}


	@Override
	protected boolean isFinished() {

		if (moments.size() >= useLast) {

			double movementChange = 0;
			double angleChange = 0;

			int size = moments.size();

			for (int i = size - (useLast - 1) - 1; i < size; i++) {
				double dX = Math.abs(moments.get(i).getX() - moments.get(i - 1).getX());
				double dY = Math.abs(moments.get(i).getY() - moments.get(i - 1).getY());
				double dAngle = Math.abs(moments.get(i).getAngle() - moments.get(i - 1).getAngle());

				movementChange += dX + dY;
				angleChange += dAngle;
			}

			// If there is less than or equal to a 10 cm change and there is less than or equal to 15 degrees of change
			return (movementChange <= movementTolerance) &&
				(angleChange <= StrictMath.toRadians(angleTolerance));
		}
		return false;
	}
//	protected boolean isFinished() {
//
//		if (robotPositions.size() >= useLast) {
//
//			double movementChange = 0;
//			double angleChange = 0;
//
//			int size = robotPositions.size();
//
//			for (int i = size - (useLast - 1) - 1; i < size; i++) {
//				double dX = Math.abs(robotPositions.get(i).getX() - robotPositions.get(i - 1).getX());
//				double dY = Math.abs(robotPositions.get(i).getY() - robotPositions.get(i - 1).getY());
//				double dAngle = Math.abs(robotPositions.get(i).getAngle() - robotPositions.get(i - 1).getAngle());
//
//				movementChange += dX + dY;
//				angleChange += dAngle;
//			}
//
//			// If there is less than or equal to a 10 cm change and there is less than or equal to 15 degrees of change
//			return movementChange <= 0.1 &&
//				angleChange <= StrictMath.toRadians(15);
//		}
//		return false;
//	}


	public List<Moment> getMoments() {
		return moments;
	}

	public String getFileName() {
		return fileName;
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
