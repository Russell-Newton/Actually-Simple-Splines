package org.waltonrobotics.motion;

import org.waltonrobotics.controller.Pose;

/**
 * <p>
 * This Path creates a straight line using the BezierCurve class.
 * </p>
 * 
 * @author Russell Newton, Walton Robotics
 * @see BezierCurve
 *
 */
public class Line extends BezierCurve {

	/**
	 * Be careful when using this. If your robot's angle is off, the MotionContoller
	 * will try to correct for it, so you will not get a straight line.
	 * 
	 * @param vCruise - the cruise velocity of the robot
	 * @param aMax - the maximum acceleration of the robot
	 * @param startVelocity - the start velocity
	 * @param endVelocity - the end velocity
	 * @param isBackwards - whether or not to move the robot backwards
	 * @param startPose - the starting Pose. Angle doesn't matter
	 * @param endPose - the ending Pose. Angle doesn't matter
	 */
	public Line(double vCruise, double aMax, double v0, double v1, boolean isBackwards, Pose startPose, Pose endPose) {
		super(vCruise, aMax, v0, v1, isBackwards, startPose, endPose);
	}
	
	/*
	 * Evan this is probably the silliest class I've ever written, but here you go.
	 */

}
