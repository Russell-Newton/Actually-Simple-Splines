package org.waltonrobotics;

import org.waltonrobotics.controller.Point;
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
	private static Point[] points = new Point[] { new Point(0, 0), new Point(1, 1) };
	private static boolean isBackwards = false;

	public static void main(String[] args) {
		System.out.println("Bezier Curve:");
		BezierCurve curve = new BezierCurve(1, 1, 0, 0, width, isBackwards, points);
		Point[] centerPoints = curve.getPathPoints();
		Point[] leftPoints = curve.getLeftPath();
		Point[] rightPoints = curve.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			// printValues(leftPoints[i].getX(), leftPoints[i].getY(),
			// rightPoints[i].getX(), rightPoints[i].getY(),
			// centerPoints[i].getDerivative(), leftPoints[i].getVelocity(),
			// rightPoints[i].getVelocity(),
			// rightPoints[i].getAcceleration());
			System.out.printf(
					"lL:%01.03f lA:%01.03f lV:%01.03f rL:%01.03f rA:%01.03f rV:%01.03f t:%01.03f  dt:%01.03f \n",
					leftPoints[i].getLength(), leftPoints[i].getAcceleration(), leftPoints[i].getVelocity(),
					rightPoints[i].getLength(), rightPoints[i].getAcceleration(), rightPoints[i].getVelocity(),
					leftPoints[i].getTime(), centerPoints[i].getAngle());
		}
		System.out.println("Spline:");
		Spline spline = new Spline(1, 1, width, 0, -90, isBackwards, points);
		centerPoints = spline.getPathPoints();
		leftPoints = spline.getLeftPath();
		rightPoints = spline.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			System.out.println(centerPoints[i].getX() + " " + centerPoints[i].getY() + " "
					+ Math.toDegrees(centerPoints[i].getAngle()));
			System.out.printf(
					"lL:%01.03f lA:%01.03f lV:%01.03f rL:%01.03f rA:%01.03f rV:%01.03f t:%01.03f \n\n",
					leftPoints[i].getLength(), leftPoints[i].getAcceleration(), leftPoints[i].getVelocity(),
					rightPoints[i].getLength(), rightPoints[i].getAcceleration(), rightPoints[i].getVelocity(),
					leftPoints[i].getTime());
		}
	}
}
