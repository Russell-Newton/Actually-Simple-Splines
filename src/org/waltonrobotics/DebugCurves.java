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
	private static double width = 0.25;
	private static Point[] points = new Point[] { new Point(0, 0), new Point(1, 0) };
	private static boolean isBackwards = true;

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
			System.out.printf("%01.03f %01.03f %01.03f %01.03f %01.03f %01.03f %01.03f \n", leftPoints[i].getLength(),
					leftPoints[i].getAcceleration(), leftPoints[i].getVelocity(), rightPoints[i].getLength(),
					rightPoints[i].getAcceleration(), rightPoints[i].getVelocity(), leftPoints[i].getTime());
		}
		System.out.println("Spline:");
		Spline spline = new Spline(1, 1, width, 0, 0, isBackwards, points);
		centerPoints = spline.getPathPoints();
		leftPoints = spline.getLeftPath();
		rightPoints = spline.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			// printValues(leftPoints[i].getX(), leftPoints[i].getY(),
			// rightPoints[i].getX(), rightPoints[i].getY(),
			// centerPoints[i].getDerivative(), leftPoints[i].getVelocity(),
			// rightPoints[i].getVelocity(),
			// leftPoints[i].getAcceleration());
			System.out.printf("%01.03f %01.03f %01.03f %01.03f %01.03f %01.03f %01.03f \n", leftPoints[i].getLength(),
					leftPoints[i].getAcceleration(), leftPoints[i].getVelocity(), rightPoints[i].getLength(),
					rightPoints[i].getAcceleration(), rightPoints[i].getVelocity(), leftPoints[i].getTime());
		}
	}

	@SuppressWarnings("unused")
	private static void printValues(double xL, double yL, double xR, double yR, double dydx, double vL, double vR,
			double a) {
		System.out.printf(
				"xL: %01.03f \t yL: %01.03f \t xR: %01.03f \t yR: %01.03f \t dy/dx: %01.03f \t vL: %01.03f \t vR: %01.03f \t a: %01.03f \n",
				xL, yL, xR, yR, dydx, vL, vR, a);
	}
}
