package org.waltonrobotics.motion;

import org.waltonrobotics.controller.Pose;

/**
 * <p>
 * This Path creates a straight line using the BezierCurve class.
 * </p>
 *
 * @author Russell Newton, Walton Robotics
 * @see BezierCurve
 */
public class Line extends BezierCurve {

	/**
	 * Be careful when using this. If your robot's angle is off, the MotionContoller will try to correct for it, so you
	 * will not get a straight line.
	 *
	 * @param vCruise - the cruise velocity of the robot
	 * @param aMax - the maximum acceleration of the robot
	 * @param startVelocity - the start velocity
	 * @param endVelocity - the end velocity
	 * @param isBackwards - whether or not to move the robot backwards
	 * @param startPose - the starting Pose. Angle doesn't matter
	 * @param endPose - the ending Pose. Angle doesn't matter
	 */
	public Line(double vCruise, double aMax, double startVelocity, double endVelocity, boolean isBackwards,
		Pose startPose, Pose endPose) {
		super(vCruise, aMax, startVelocity, endVelocity, isBackwards, startPose, endPose);
	}

	/**
	 * Be careful when using this. If your robot's angle is off, the MotionContoller will try to correct for it, so you
	 * will not get a straight line.
	 *
	 * @param vCruise - the cruise velocity of the robot
	 * @param aMax - the maximum acceleration of the robot
	 * @param startVelocity - the start velocity
	 * @param endVelocity - the end velocity
	 * @param isBackwards - whether or not to move the robot backwards
	 * @param startPose - the starting Pose. Angle doesn't matter
	 * @param distance - how you wish to offset the startingPose from.
	 */
	public Line(double vCruise, double aMax, double startVelocity, double endVelocity, boolean isBackwards,
		Pose startPose, double distance) {
		this(vCruise, aMax, startVelocity, endVelocity, isBackwards, startPose, startPose.offset(distance));
	}
}
