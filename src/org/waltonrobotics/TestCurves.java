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
public class TestCurves {

	// Change these to see their effect
	public static int steps = 100;
	public static double width = 0.25;
	private static Point[] points = new Point[] { new Point(0, 0), new Point(1, 0) };

	public static void main(String[] args) {
		System.out.println("Bezier Curve:");
		BezierCurve curve = new BezierCurve(.5, .5, 0, 0, steps, width, points);
		Point[] centerPoints = curve.getPathPoints();
		Point[] leftPoints = curve.getLeftPath();
		Point[] rightPoints = curve.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
//			printValues(leftPoints[i].getX(), leftPoints[i].getY(), rightPoints[i].getX(), rightPoints[i].getY(),
//					centerPoints[i].getDerivative(), leftPoints[i].getVelocity(), rightPoints[i].getVelocity(),
//					rightPoints[i].getAcceleration());
			System.out.printf("%01.03f %01.03f %01.03f %01.03f %01.03f %01.03f %01.03f %01.03f \n",
					leftPoints[i].getLength(), leftPoints[i].getAcceleration(), leftPoints[i].getVelocity(),
					rightPoints[i].getLength(), rightPoints[i].getAcceleration(), rightPoints[i].getVelocity(),
					leftPoints[i].getTime(), rightPoints[i].getTime());
		}
		System.out.println("Spline:");
		Spline spline = new Spline(.5, .5, width, 0, 0, points);
		centerPoints = spline.getPathPoints();
		leftPoints = spline.getLeftPath();
		rightPoints = spline.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			// printValues(leftPoints[i].getX(), leftPoints[i].getY(),
			// rightPoints[i].getX(), rightPoints[i].getY(),
			// centerPoints[i].getDerivative(), leftPoints[i].getVelocity(),
			// rightPoints[i].getVelocity(),
			// leftPoints[i].getAcceleration());
			System.out.printf("%01.03f %01.03f %01.03f %01.03f %01.03f %01.03f %01.03f %01.03f \n",
					leftPoints[i].getLength(), leftPoints[i].getAcceleration(), leftPoints[i].getVelocity(),
					rightPoints[i].getLength(), rightPoints[i].getAcceleration(), rightPoints[i].getVelocity(),
					leftPoints[i].getTime(), rightPoints[i].getTime());
		}
	}

	private static void printValues(double xL, double yL, double xR, double yR, double dydx, double vL, double vR,
			double a) {
		System.out.printf(
				"xL: %01.03f \t yL: %01.03f \t xR: %01.03f \t yR: %01.03f \t dy/dx: %01.03f \t vL: %01.03f \t vR: %01.03f \t a: %01.03f \n",
				xL, yL, xR, yR, dydx, vL, vR, a);
	}
}
