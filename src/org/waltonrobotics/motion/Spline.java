package org.waltonrobotics.motion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.Point;

/**
 * This path is a spline that will go through the set knots by stitching
 * together several Bezier curves. By default, it will try to make the shortest
 * path possible, but the start and end angles (degrees) indicate how the robot
 * is facing or how you want it to face. This is not very effective with only 2
 * knots. If you want a straight line, make a Bezier Curve.
 * 
 * @see {@link https://www.particleincell.com/2012/bezier-splines/}
 * @see {@link https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm}
 * 
 * @author Russell Newton, Walton Robotics
 *
 */

public class Spline extends Path {

	public double robotWidth;

	private List<List<Point>> pathControlPoints;
	private Point[] pathPoints;
	private Point[] leftPoints;
	private Point[] rightPoints;
	private final double startAngle;
	private final double endAngle;
	private final boolean isBackwards;

	/**
	 * Construct a spline. Note that the x axis is the direction the robot is facing
	 * if the start angle is 0
	 * 
	 * @param vCruise
	 *            - max velocity
	 * @param aMax
	 *            - max acceleration
	 * @param robotWidth
	 *            - the width of the robot, should be in the same unit as the
	 *            encoder distance per tick
	 * @param startAngle
	 *            - the angle at the start of the motion (degrees)
	 * @param endAngle
	 *            - the angle at the end of the motion (degrees)
	 * @param isBackwards
	 *            - if the robot will be moving backwards, make this true
	 * @param knots
	 *            - the points you want the robot to drive through
	 */
	public Spline(double vCruise, double aMax, double robotWidth, double startAngle, double endAngle,
			boolean isBackwards, Point... knots) {
		super(vCruise, aMax);
		this.robotWidth = robotWidth;
		this.startAngle = startAngle;
		this.endAngle = endAngle;
		this.isBackwards = isBackwards;
		pathControlPoints = computeControlPoints(knots);
		joinBezierCurves(pathControlPoints);
	}

	/**
	 * Creates the control points required to make cubic bezier curves that
	 * transition between knots. Will make them for the shortest path possible.
	 * 
	 * @param knots
	 * @return A list of lists that hold the control points for the segments in the
	 *         spline
	 */
	private List<List<Point>> computeControlPoints(Point[] knots) {
		int degree = knots.length - 1;
		Point[] points1 = new Point[degree];
		Point[] points2 = new Point[degree];

		/* constants for Thomas Algorithm */
		double[] a = new double[degree];
		double[] b = new double[degree];
		double[] c = new double[degree];
		double[] r_x = new double[degree];
		double[] r_y = new double[degree];

		/* left most segment */
		a[0] = 0;
		b[0] = 2;
		c[0] = 1;
		r_x[0] = knots[0].getX() + 2 * knots[1].getX();
		r_y[0] = knots[0].getY() + 2 * knots[1].getY();

		/* internal segments */
		for (int i = 1; i < degree - 1; i++) {
			a[i] = 1;
			b[i] = 4;
			c[i] = 1;
			r_x[i] = 4 * knots[i].getX() + 2 * knots[i + 1].getX();
			r_y[i] = 4 * knots[i].getY() + 2 * knots[i + 1].getY();
		}

		/* right segment */
		a[degree - 1] = 2;
		b[degree - 1] = 7;
		c[degree - 1] = 0;
		r_x[degree - 1] = 8 * knots[degree - 1].getX() + knots[degree].getX();
		r_y[degree - 1] = 8 * knots[degree - 1].getY() + knots[degree].getY();

		/* solves Ax=b with the Thomas algorithm */
		for (int i = 1; i < degree; i++) {
			double m = a[i] / b[i - 1]; // temporary variable
			b[i] = b[i] - m * c[i - 1];
			r_x[i] = r_x[i] - m * r_x[i - 1];
			r_y[i] = r_y[i] - m * r_y[i - 1];
		}
		points1[degree - 1] = new Point(r_x[degree - 1] / b[degree - 1], r_y[degree - 1] / b[degree - 1]);
		for (int i = degree - 2; i >= 0; --i) {
			points1[i] = new Point((r_x[i] - c[i] * points1[i + 1].getX()) / b[i],
					(r_y[i] - c[i] * points1[i + 1].getY()) / b[i]);
		}

		/* we have p1, now compute p2 */
		for (int i = 0; i < degree - 1; i++) {
			points2[i] = new Point(2 * knots[i + 1].getX() - points1[i + 1].getX(),
					2 * knots[i + 1].getY() - points1[i + 1].getY());
		}

		points2[degree - 1] = new Point(0.5 * (knots[degree].getX() + points1[degree - 1].getX()),
				0.5 * (knots[degree].getY() + points1[degree - 1].getY()));
		List<List<Point>> controlPoints = new ArrayList<>();
		for (int i = 0; i < degree; i++) {
			List<Point> segmentControlPoints = new ArrayList<>();
			Collections.addAll(segmentControlPoints, knots[i], points1[i], points2[i], knots[i + 1]);
			Collections.addAll(controlPoints, segmentControlPoints);
		}

		return controlPoints;
	}

	/**
	 * Joins the bezier curves defining the spline into intdividual arrays of Points
	 * 
	 * @param pathControlPoints
	 *            - a List of Lists of control points for each curve that make up
	 *            the spline
	 */
	private void joinBezierCurves(List<List<Point>> pathControlPoints) {
		List<Point> pathPointsAdd = new ArrayList<>();
		List<Point> leftPointsAdd = new ArrayList<>();
		List<Point> rightPointsAdd = new ArrayList<>();

		for (int i = 0; i < pathControlPoints.size(); i++) {
			Point[] controlPoints = pathControlPoints.get(i).stream().toArray(Point[]::new);
			// Change the second control point to get the start angle to always be 0. This
			// way, the robot will always start by going forwards. We can do this, because
			// the start derivative = the slope between the first two control points.
			if (i == 0) {
				controlPoints[1] = controlPoints[1].rotate(controlPoints[0], startAngle);
			}
			// Change the second to last control point to get the desired end angle. We can
			// do this, because the end derivative = the slope between the last two control
			// points
			if (i == pathControlPoints.size() - 1) {
				controlPoints[controlPoints.length - 2] = controlPoints[controlPoints.length - 2]
						.rotate(controlPoints[controlPoints.length - 1], endAngle);
			}
			BezierCurve curve = new BezierCurve(vCruise, aMax, i != 0 ? vCruise : 0,
					i != pathControlPoints.size() - 1 ? vCruise : 0, robotWidth, isBackwards, controlPoints);

			Point[] pathPoints;
			Point[] leftPoints;
			Point[] rightPoints;
			if (i != pathControlPoints.size() - 1) {

				pathPoints = curve.getPathPoints();
				pathPoints = Arrays.copyOfRange(pathPoints, 0, pathPoints.length - 2);
				leftPoints = curve.getLeftPath();
				leftPoints = Arrays.copyOfRange(leftPoints, 0, leftPoints.length - 2);
				rightPoints = curve.getRightPath();
				rightPoints = Arrays.copyOfRange(rightPoints, 0, rightPoints.length - 2);
			} else {
				pathPoints = curve.getPathPoints();
				leftPoints = curve.getLeftPath();
				rightPoints = curve.getRightPath();
			}
			Collections.addAll(pathPointsAdd, pathPoints);
			Collections.addAll(leftPointsAdd, leftPoints);
			Collections.addAll(rightPointsAdd, rightPoints);
		}
		this.pathPoints = pathPointsAdd.stream().toArray(Point[]::new);
		this.leftPoints = leftPointsAdd.stream().toArray(Point[]::new);
		this.rightPoints = rightPointsAdd.stream().toArray(Point[]::new);
	}

	@Override
	public Point[] getPathPoints() {
		return pathPoints;
	}

	@Override
	public Point[] getLeftPath() {
		return leftPoints;
	}

	@Override
	public Point[] getRightPath() {
		return rightPoints;
	}

}
