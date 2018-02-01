package org.waltonrobotics.motion;

import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.Point;
import org.waltonrobotics.controller.RobotPair;
import org.waltonrobotics.controller.State;

/**
 * This Path is a simple curve. The shape of the curve is controlled by the
 * control points. There are tangents from the first and second control points
 * and the second to last and last control points. Use this to make a straight
 * line (use two control points)
 * 
 * @see {@link https://en.wikipedia.org/wiki/B%C3%A9zier_curve}
 * @see {@link https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html}
 * @author Marius Juston, Walton Robotics
 * @author Russell Newton, Walton Robotics
 */
public class BezierCurve extends Path {

	private final double startVelocity;
	private final double endVelocity;
	private final double startLCenter;
	private double curveLength = 0;
	private final int numberOfSteps;
	private final boolean isBackwards;
	private final RobotPair lastPair;
	private final double startTime;

	private final Point[] pathPoints;
	private final Point[] leftPoints;
	private final Point[] rightPoints;

	private final double robotLength;
	private final Point[] controlPoints;
	private double[] coefficients;

	/**
	 * This constructor is used with the splines, but feel free to use it when
	 * creating your own motions
	 * 
	 * @param vCruise
	 *            - the cruise velocity of the robot
	 * @param aMax
	 *            - the maximum acceleration of the robot
	 * @param v0
	 *            - the start velocity
	 * @param v1
	 *            - the end velocity
	 * @param robotWidth
	 *            - the width of the robot
	 * @param startTime
	 *            - the starting time of the motion
	 * @param controlPoints
	 *            - the control points that define the robot
	 */
	public BezierCurve(double vCruise, double aMax, double v0, double v1, double robotWidth, boolean isBackwards,
			RobotPair lastPair, double startTime, Point... controlPoints) {
		super(vCruise, aMax);
		this.robotLength = robotWidth;
		this.controlPoints = controlPoints;
		this.numberOfSteps = 50;
		this.isBackwards = isBackwards;
		this.lastPair = lastPair;
		this.startTime = startTime;
		startVelocity = v0;
		endVelocity = v1;
		// The starting average encoder distance should always be 0
		startLCenter = 0;

		updateCoefficients();
		pathPoints = getCurvePoints(numberOfSteps, controlPoints);

		rightPoints = offsetPoints(pathPoints)[2];
		leftPoints = offsetPoints(pathPoints)[0];
	}

	public BezierCurve(double vCruise, double aMax, double v0, double v1, double robotWidth, boolean isBackwards,
			Point... controlPoints) {
		super(vCruise, aMax);
		this.robotLength = robotWidth;
		this.controlPoints = controlPoints;
		this.numberOfSteps = 50;
		this.isBackwards = isBackwards;
		this.lastPair = new RobotPair(0, 0);
		startTime = 0;
		startVelocity = v0;
		endVelocity = v1;
		startLCenter = (lastPair.getLeft() + lastPair.getRight()) / 2;

		updateCoefficients();
		pathPoints = getCurvePoints(numberOfSteps, controlPoints);

		rightPoints = offsetPoints(pathPoints)[2];
		leftPoints = offsetPoints(pathPoints)[0];
	}

	/**
	 * Uses the formula to find the value of nCr
	 * 
	 * @param n
	 * @param r
	 * @return nCr
	 */
	private static double findNumberOfCombination(double n, double r) {
		double nFactorial = factorial(n);
		double rFactorial = factorial(r);
		double nMinusRFactorial = factorial(n - r);

		return nFactorial / (rFactorial * nMinusRFactorial);
	}

	/**
	 * Finds the factorial of any integer or double, d
	 * 
	 * @param d
	 * @return the factorial of d
	 */
	private static double factorial(double d) {
		double r = d - Math.floor(d) + 1;
		for (; d > 1; d -= 1) {
			r *= d;
		}
		return r;
	}

	/**
	 * @param numberOfSteps
	 * @param controlPoints
	 * @return an array of Points that define the curve
	 */
	private Point[] getCurvePoints(int numberOfSteps, Point[] controlPoints) {
		Point[] point2DList = new Point[numberOfSteps + 1];

		for (double i = 0; i <= numberOfSteps; i++) {
			point2DList[(int) i] = getPoint(i / ((double) numberOfSteps), controlPoints);
			if (i != 0)
				curveLength += point2DList[(int) i].distance(point2DList[(int) i - 1]);
		}

		return point2DList;
	}

	/**
	 * Updates the coefficients used for calculations
	 */
	private void updateCoefficients() {
		int n = getDegree();
		coefficients = new double[n + 1];
		for (int i = 0; i < coefficients.length; i++) {
			coefficients[i] = findNumberOfCombination(n, i);
		}
	}

	/**
	 * Returns the point on the curve at any percentage on the line, t
	 * 
	 * @param percentage
	 *            - t
	 * @param controlPoints
	 * @return the Point that is at percentage t along the curve
	 */
	private Point getPoint(double percentage, Point[] controlPoints) {
		double xCoordinateAtPercentage = 0;
		double yCoordinateAtPercentage = 0;

		int n = getDegree();

		for (int i = 0; i <= n; i++) {
			double coefficient = coefficients[i];

			double oneMinusT = Math.pow(1 - percentage, (double)(n - i));

			double powerOfT = Math.pow(percentage, (double)i);

			Point pointI = controlPoints[i];

			xCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getX());
			yCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getY());
		}

		return new Point(xCoordinateAtPercentage, yCoordinateAtPercentage, getAngle(percentage));
	}

	/**
	 * @return the degree of the curve
	 */
	private int getDegree() {
		return controlPoints.length - 1;
	}

	/**
	 * Given the control points defining the curve, find the derivative at any point
	 * on the curve
	 * 
	 * @param t
	 *            - percent along curve
	 * @param controlPoints
	 * @return derivative at point
	 */
	private double getAngle(double t) {
		int n = getDegree();
		double dx = 0;
		double dy = 0;
		for (int i = 0; i < n; i++) {
			double coefficient = findNumberOfCombination(n, i) * Math.pow(t, i) * Math.pow(1 - t, n - i);
			dx += coefficient * (n + 1) * (controlPoints[i + 1].getX() - controlPoints[i].getX());
			dy += coefficient * (n + 1) * (controlPoints[i + 1].getY() - controlPoints[i].getY());
		}
		if(t == 1) {
			dx = controlPoints[controlPoints.length - 1].getX() - controlPoints[controlPoints.length - 2].getX();
			dy = controlPoints[controlPoints.length - 1].getY() - controlPoints[controlPoints.length - 2].getY();
		}
		System.out.println(dx + " " + dy);
		double angle = Math.atan2(dy, dx);
		return angle;
	}

	/**
	 * Offsets control points of a curve
	 * 
	 * @param pathPoints
	 * @param isRightSide
	 * @return an array of Points that defines an offset curve
	 */
	private Point[][] offsetPoints(Point[] pathPoints) {
		int n = pathPoints.length;
		Point[] offsetPointsCenter = new Point[n];
		Point[] offsetPointsLeft = new Point[n];
		Point[] offsetPointsRight = new Point[n];
		for (int i = 0; i < n; i++) {

			Point[] calculatedPoints = new Point[3];
			if (i == 0) {
				Point center = new Point(pathPoints[i].getX(), pathPoints[i].getY(), pathPoints[i].getAngle(),
						null, startLCenter, startTime);
				Point left = new Point(0, 0, 0, new State(lastPair.getLeft(), startVelocity, aMax), 0, 0);
				Point right = new Point(0, 0, 0, new State(lastPair.getRight(), startVelocity, aMax), 0, 0);
				calculatedPoints = new Point[] { left, center, right };
			} else {
				calculatedPoints = calculatePoints(offsetPointsLeft[i - 1], offsetPointsCenter[i - 1],
						offsetPointsRight[i - 1], pathPoints[i], i);
			}

			offsetPointsLeft[i] = calculatedPoints[0];
			offsetPointsCenter[i] = calculatedPoints[1];
			offsetPointsRight[i] = calculatedPoints[2];
		}
		return new Point[][] { offsetPointsLeft, offsetPointsCenter, offsetPointsRight };
	}

	/**
	 * Calculates the velocities and acceleration required to get from one point to
	 * the next. I gotta be honest I have no idea how this math works - Russell
	 * 
	 * @param previousPoint
	 * @param currentCenter
	 * @param i
	 *            - the step number
	 * @return the wheel velocities acceleration, lCenter, and the time to get to
	 *         the next point
	 */
	private Point[] calculatePoints(Point previousLeft, Point previousCenter, Point previousRight, Point currentCenter,
			int i) {

		// When cruising, acceleration is 0
		double acceleration = 0;

		// The change in angle of the robot
		double dAngle = currentCenter.getAngle() - previousCenter.getAngle();

		// The change in distance of the robot sides
		double dLength = previousCenter.distance(currentCenter) * (isBackwards? -1 : 1);
		double dlLeft = dLength - dAngle * robotLength / 2;
		double dlRight = dLength + dAngle * robotLength / 2;

		// The time required to get to the next point
		double dTime = Math.max(Math.abs(dlLeft), Math.abs(dlRight)) / vCruise;

		// The hypothetical velocity to get to that point
		double velocity = Math.abs(dLength) / dTime;

		// The average encoder distance to the next point
		double lCenter = previousCenter.getLCenter() + 0.5 * dLength - startLCenter;

		double vAccelerating = Math.sqrt(Math.pow(startVelocity, 2) + aMax * Math.abs(lCenter));
		double vDecelerating = Math.sqrt(Math.pow(endVelocity, 2) + aMax * Math.abs(curveLength - lCenter));
		System.out.println("acc: " + vAccelerating + " dec: " + vDecelerating + " vel: " + velocity);
		if (vAccelerating < velocity && vAccelerating < vDecelerating) {
			acceleration = aMax;
			dTime = Math.abs(dLength) / vAccelerating;
		}
		if (vDecelerating < velocity && vDecelerating < vAccelerating) {
			acceleration = -aMax;
			dTime = Math.abs(dLength) / vDecelerating;
		}
		double velocityL = dlLeft / dTime;
		double velocityR = dlRight / dTime;

		if (isBackwards) {
			velocityL *= -1;
			velocityR *= -1;
			dlLeft *= -1;
			dlRight *= -1;
		}

		double newLCenter = previousCenter.getLCenter() + (dlLeft + dlRight) / 2;

		Point center = new Point(currentCenter.getX(), currentCenter.getY(), currentCenter.getAngle(),
				new State(0, 0, 0), newLCenter, previousCenter.getTime() + dTime);
		Point left = center.offsetPerpendicular(-robotLength / 2,
				new State(previousLeft.getLength() + dlLeft, velocityL, acceleration), newLCenter,
				previousCenter.getTime() + dTime);
		Point right = center.offsetPerpendicular(robotLength / 2,
				new State(previousRight.getLength() + dlRight, velocityR, acceleration), newLCenter,
				previousCenter.getTime() + dTime);

		return new Point[] { left, center, right };
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
