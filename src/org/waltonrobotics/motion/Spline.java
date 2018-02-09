package org.waltonrobotics.motion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.ListIterator;

import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.State;

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

	private List<List<Pose>> pathControlPoints;
	private List<PathData> pathData;
	private final double startAngle;
	private final double endAngle;
	private final boolean isBackwards;
	private final double startVelocity;
	private final double endVelocity;

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
	public Spline(double vCruise, double aMax, double startVelocity, double endVelocity, double robotWidth,
			double startAngle, double endAngle, boolean isBackwards, List<Pose> knots) {
		super(vCruise, aMax, robotWidth);
		this.startAngle = startAngle;
		this.endAngle = endAngle;
		this.isBackwards = isBackwards;
		this.endVelocity = endVelocity;
		this.startVelocity = startVelocity;
		pathControlPoints = computeControlPoints(knots);
		PathData startPathData = new PathData(new State(0, startVelocity, 0), new State(0, startVelocity, 0),
				pathControlPoints.get(0).get(0), 0);
		pathData = new ArrayList<PathData>();
		stitchPathData(startPathData);
	}

	/**
	 * Creates the control points required to make cubic bezier curves that
	 * transition between knots. Will make them for the shortest path possible.
	 * 
	 * @param knots
	 * @return A list of lists that hold the control points for the segments in the
	 *         spline
	 */
	private List<List<Pose>> computeControlPoints(List<Pose> knots) {
		int degree = knots.size() - 1;
		Pose[] points1 = new Pose[degree];
		Pose[] points2 = new Pose[degree];

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
		r_x[0] = knots.get(0).getX() + 2 * knots.get(1).getX();
		r_y[0] = knots.get(0).getY() + 2 * knots.get(1).getY();

		/* internal segments */
		for (int i = 1; i < degree - 1; i++) {
			a[i] = 1;
			b[i] = 4;
			c[i] = 1;
			r_x[i] = 4 * knots.get(i).getX() + 2 * knots.get(i + 1).getX();
			r_y[i] = 4 * knots.get(i).getY() + 2 * knots.get(i + 1).getY();
		}

		/* right segment */
		a[degree - 1] = 2;
		b[degree - 1] = 7;
		c[degree - 1] = 0;
		r_x[degree - 1] = 8 * knots.get(degree - 1).getX() + knots.get(degree).getX();
		r_y[degree - 1] = 8 * knots.get(degree - 1).getY() + knots.get(degree).getY();

		/* solves Ax=b with the Thomas algorithm */
		for (int i = 1; i < degree; i++) {
			double m = a[i] / b[i - 1]; // temporary variable
			b[i] = b[i] - m * c[i - 1];
			r_x[i] = r_x[i] - m * r_x[i - 1];
			r_y[i] = r_y[i] - m * r_y[i - 1];
		}
		points1[degree - 1] = new Pose(r_x[degree - 1] / b[degree - 1], r_y[degree - 1] / b[degree - 1]);
		for (int i = degree - 2; i >= 0; --i) {
			points1[i] = new Pose((r_x[i] - c[i] * points1[i + 1].getX()) / b[i],
					(r_y[i] - c[i] * points1[i + 1].getY()) / b[i]);
		}

		/* we have p1, now compute p2 */
		for (int i = 0; i < degree - 1; i++) {
			points2[i] = new Pose(2 * knots.get(i + 1).getX() - points1[i + 1].getX(),
					2 * knots.get(i + 1).getY() - points1[i + 1].getY());
		}

		points2[degree - 1] = new Pose(0.5 * (knots.get(degree).getX() + points1[degree - 1].getX()),
				0.5 * (knots.get(degree).getY() + points1[degree - 1].getY()));
		List<List<Pose>> controlPoints = new ArrayList<>();
		for (int i = 0; i < degree; i++) {
			List<Pose> segmentControlPoints = new ArrayList<>();
			points1[0] = points1[0].rotate(knots.get(0), startAngle, isBackwards);
			points2[degree - 1] = points2[degree - 1].rotate(knots.get(knots.size() - 1), endAngle, !isBackwards);
			Collections.addAll(segmentControlPoints, knots.get(i), points1[i], points2[i], knots.get(i + 1));
			Collections.addAll(controlPoints, segmentControlPoints);
		}
		return controlPoints;
	}

	/**
	 * Stitches the bezier curve path datas to make the single spline
	 * 
	 * @param startPathData
	 *            - requires the initial pathData
	 */
	private void stitchPathData(PathData startPathData) {
		double nextV0;
		double nextV1;
		PathData nextStartPathData = startPathData;
		pathData.add(startPathData);
		ListIterator<List<Pose>> iterator = pathControlPoints.listIterator();
		while (iterator.hasNext()) {
			BezierCurve curve;
			if (iterator.nextIndex() == 0) {
				nextV0 = startVelocity;
			} else {
				nextV0 = vCruise;
			}
			if (iterator.nextIndex() == pathControlPoints.size() - 1) {
				nextV1 = endVelocity;
			} else {
				nextV1 = vCruise;
			}
			curve = new BezierCurve(vCruise, aMax, nextV0, nextV1, robotWidth, isBackwards, nextStartPathData,
					iterator.next());
			pathData.addAll(curve.getPathData());
			nextStartPathData = pathData.get(pathData.size() - 1);
		}
	}

	@Override
	public List<PathData> getPathData() {
		return pathData;
	}

}
