package org.waltonrobotics.command;

import java.util.Arrays;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.Spline;

/**
 *
 */
public class SimpleSpline extends SimpleMotion {

	public SimpleSpline(double maxVelocity, double maxAcceleration, double startAngle,
		double endAngle, Pose... knots) {
		this(maxVelocity, maxAcceleration, startAngle, endAngle, false, knots);
	}

	public SimpleSpline(double maxVelocity, double maxAcceleration, double startAngle,
		double endAngle, boolean isBackwards, double startScale, double endScale, double startVelocity,
		double endVelocity, Pose... knots) {
		super(new Spline(
			maxVelocity,
			maxAcceleration,
			startVelocity,
			endVelocity,
			startAngle,
			endAngle,
			isBackwards,
			startScale,
			endScale,
			Arrays.asList(knots)));
	}

	public SimpleSpline(double maxVelocity, double maxAcceleration, double startAngle,
		double endAngle, boolean isBackwards, Pose... knots) {
		this(maxVelocity, maxAcceleration, startAngle, endAngle, isBackwards, 1,
			1, 0, 0, knots);
	}

	public static SimpleSpline pathFromPosesWithAngle(boolean isBackwards, Pose... knots) {
		return pathFromPosesWithAngle(getDrivetrain().getMaxVelocity(), getDrivetrain().getMaxAcceleration(),
			isBackwards, knots);
	}

	/**
	 * Creates a SimpleSpline where the first angle is the angle of the first point and the final angle is the angle of
	 * the last point.
	 *
	 * @param isBackwards will the robot move forwards or backwards
	 * @param knots the points (with angle) to move through
	 * @return a new SimpleSpline command
	 */
	public static SimpleSpline pathFromPosesWithAngle(double maxVelocity, double maxAcceleration,
		boolean isBackwards, Pose... knots) {
		return new SimpleSpline(maxVelocity, maxAcceleration, knots[0].getAngle(),
			knots[knots.length - 1].getAngle(), isBackwards, knots);
	}


	public static SimpleSpline pathFromPosesWithAngle(double maxVelocity, double maxAcceleration,
		double startVelocity, double endVelocity, boolean isBackwards, Pose... knots) {
		return new SimpleSpline(maxVelocity, maxAcceleration, knots[0].getAngle(),
			knots[knots.length - 1].getAngle(), isBackwards, 1, 1, startVelocity, endVelocity, knots);
	}


	public static SimpleSpline pathFromPosesWithAngleAndScale(double maxVelocity,
		double maxAcceleration,
		boolean isBackwards, double startScale, double endScale, Pose... knots) {
		return new SimpleSpline(maxVelocity, maxAcceleration, knots[0].getAngle(),
			knots[knots.length - 1].getAngle(), isBackwards, startScale, endScale, 0, 0, knots);
	}

	public static SimpleSpline pathFromPosesWithAngleAndScale(
		boolean isBackwards, double startScale, double endScale, Pose... knots) {
		return pathFromPosesWithAngleAndScale(getDrivetrain().getMaxVelocity(), getDrivetrain().getMaxAcceleration(),
			isBackwards,
			startScale, endScale, knots);
	}

}
