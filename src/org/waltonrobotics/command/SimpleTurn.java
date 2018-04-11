package org.waltonrobotics.command;

import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.PointTurn;

public class SimpleTurn extends SimpleMotion {

	public SimpleTurn(double maxVelocity, double maxAcceleration, Pose startPosition,
		double endAngle) {
		super(new PointTurn(
			maxVelocity,
			maxAcceleration,
			startPosition,
			endAngle));
	}

	public static SimpleTurn pointTurn(Pose startPosition, double endAngle) {
		return pointTurn(getDrivetrain().getMaxVelocity() / 4, getDrivetrain().getMaxAcceleration(), startPosition,
			endAngle);
	}

	public static SimpleTurn pointTurn(double maxVelocity, double maxAcceleration,
		Pose startPosition, double endAngle) {
		return new SimpleTurn(maxVelocity, maxAcceleration, startPosition, endAngle);
	}

	public static SimpleTurn pointTurn(Pose startPosition, Pose endPosition) {
		return pointTurn(getDrivetrain().getMaxVelocity() / 2, getDrivetrain().getMaxAcceleration(), startPosition,
			endPosition.getAngle());
	}
}
