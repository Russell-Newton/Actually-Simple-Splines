package org.waltonrobotics;

import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.State;
import org.waltonrobotics.motion.BezierCurve;
import org.waltonrobotics.motion.Spline;

/**
 * Run this class to see how changing points affects the curves
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class DebugCurves {

	// Change these to see their effect
	private static double width = .70485;
	private static Pose[] points = new Pose[] { new Pose(0, 0), new Pose(.5, .5), new Pose(1, 0) };
	private static boolean isBackwards = false;

	public static void main(String[] args) {
		System.out.println("Bezier Curve:");
		BezierCurve curve = new BezierCurve(1, 1, 0, 0, width, isBackwards, points);
		Pose[] centerPoses = curve.getPathData().getCenterPoses();
		State[] leftStates = curve.getPathData().getLeftStates();
		State[] rightStates = curve.getPathData().getRightStates();
		double[] times = curve.getPathData().getTimes();
		for (int i = 0; i < centerPoses.length; i++) {
			System.out.println(centerPoses[i].getX() + " " + centerPoses[i].getY() + " "
					+ Math.toDegrees(centerPoses[i].getAngle()));
			// System.out.printf(
			// "lL:%01.03f lA:%01.03f lV:%01.03f rL:%01.03f rA:%01.03f rV:%01.03f t:%01.03f
			// \n",
			// leftStates[i].getLength(), leftStates[i].getAcceleration(),
			// leftStates[i].getVelocity(),
			// rightStates[i].getLength(), rightStates[i].getAcceleration(),
			// rightStates[i].getVelocity(),
			// times[i]);
		}
		System.out.println("Spline:");
		Spline spline = new Spline(1, 1, width, 0, 0, isBackwards, points);
		centerPoses = spline.getPathData().getCenterPoses();
		leftStates = spline.getPathData().getLeftStates();
		rightStates = spline.getPathData().getRightStates();
		times = spline.getPathData().getTimes();
		for (int i = 0; i < centerPoses.length; i++) {
			System.out.println(centerPoses[i].getX() + " " + centerPoses[i].getY() + " "
					+ Math.toDegrees(centerPoses[i].getAngle()));
			// System.out.printf(
			// "lL:%01.03f lA:%01.03f lV:%01.03f rL:%01.03f rA:%01.03f rV:%01.03f t:%01.03f
			// \n",
			// leftStates[i].getLength(), leftStates[i].getAcceleration(),
			// leftStates[i].getVelocity(),
			// rightStates[i].getLength(), rightStates[i].getAcceleration(),
			// rightStates[i].getVelocity(),
			// times[i]);
		}
	}
}
