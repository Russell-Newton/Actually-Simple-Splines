package org.waltonrobotics.motion;

import java.util.LinkedList;
import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.State;

public class PointTurn extends Path {

	private final Pose startPose;
	private final double endAngle;
	private final double turnStep;
	private final double vRotationMax;
	private final double aRotationMax;
	private final LinkedList<PathData> pathData;

	public PointTurn(double vRotationMax, double aRotationMax, Pose startPosition,
		double endAngle) {
		super(vRotationMax, aRotationMax, false,
			startPosition, new Pose(startPosition.getX(), startPosition.getY(), endAngle));
		startPose = startPosition;
		this.endAngle = endAngle;

		this.vRotationMax = vRotationMax;
		this.aRotationMax = aRotationMax;
		double turnAngle = this.endAngle - startPose.getAngle();
		turnAngle = boundAngle(turnAngle);

		turnStep = turnAngle / getPathNumberOfSteps();
		pathData = new LinkedList<>();
		generateData();
	}

	private void generateData() {
		pathData.add(getPathData(new PathData(startPose)));
		for (int i = 1; i < getPathNumberOfSteps(); i++) {
			pathData.add(getPathData(pathData.getLast()));
		}
	}

	private PathData getPathData(PathData previous) {

		// estimate lengths each wheel will turn
		double dlLeft = ((-turnStep * getRobotWidth()) / 2.0);
		double dlRight = ((turnStep * getRobotWidth()) / 2.0);

		// assuming one of the wheels will limit motion, calculate time this step will
		// take
		double dt = Math.max(Math.abs(dlLeft), Math.abs(dlRight)) / vRotationMax;
		double a = 0.0;

		// assuming constant angular acceleration from/to zero angular speed
		double omega = Math.abs(dlRight - dlLeft) / dt / getRobotWidth();
		double thetaMidpoint = previous.getCenterPose().getAngle() + (0.5 * turnStep);

		double omegaAccel = Math.sqrt(aRotationMax
			* Math.abs(boundAngle(thetaMidpoint - startPose.getAngle())));
		double omegaDecel = Math.sqrt(aRotationMax
			* Math.abs(boundAngle(thetaMidpoint - endAngle)));
//		System.out.println("OmegaAccel=" + omegaAccel);
//         System.out.println(omega);
		if ((omegaAccel < omega) && (omegaAccel < omegaDecel)) {
			dt = Math.abs(dlRight - dlLeft) / omegaAccel / getRobotWidth();
		}

//		System.out.println("OmegaDecel=" + omegaDecel);
		if ((omegaDecel < omega) && (omegaDecel < omegaAccel)) {
			dt = Math.abs(dlRight - dlLeft) / omegaDecel / getRobotWidth();
		}
//        System.out.println(dt);

		State left = new State(previous.getLeftState().getLength() + dlLeft, dlLeft / dt, a);
		State right = new State(previous.getRightState().getLength() + dlRight, dlRight / dt, a);
		Pose center = new Pose(startPose.getX(), startPose.getY(),
			previous.getCenterPose().getAngle() + turnStep);

		return new PathData(left, right, center, previous.getTime() + dt);
	}

	@Override
	public LinkedList<PathData> getPathData() {
		return pathData;
	}

	@Override
	public String toString() {
		return "PointTurn{" +
			"startPose=" + startPose +
			", endAngle=" + endAngle +
			", turnStep=" + turnStep +
			", vRotationMax=" + vRotationMax +
			", aRotationMax=" + aRotationMax +
			", pathData=" + pathData +
			"} " + super.toString();
	}
}
