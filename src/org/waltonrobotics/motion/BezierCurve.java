package org.waltonrobotics.motion;

import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
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

	private final Pose[] centerPoses;
	private final State[] leftStates;
	private final State[] rightStates;
	private final double[] times;

	private final Pose[] controlPoints;
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
			RobotPair lastPair, double startTime, Pose... controlPoints) {
		super(vCruise, aMax, robotWidth);
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
		centerPoses = getCurvePoints(numberOfSteps, controlPoints);

		rightStates = new State[numberOfSteps];
		leftStates = new State[numberOfSteps];
		times = new double[numberOfSteps];
		setData();
	}

	public BezierCurve(double vCruise, double aMax, double v0, double v1, double robotWidth, boolean isBackwards,
			Pose... controlPoints) {
		super(vCruise, aMax, robotWidth);
		this.controlPoints = controlPoints;
		this.numberOfSteps = 50;
		this.isBackwards = isBackwards;
		this.lastPair = new RobotPair(0, 0);
		startTime = 0;
		startVelocity = v0;
		endVelocity = v1;
		startLCenter = (lastPair.getLeft() + lastPair.getRight()) / 2;

		updateCoefficients();
		centerPoses = getCurvePoints(numberOfSteps, controlPoints);

		rightStates = new State[numberOfSteps];
		leftStates = new State[numberOfSteps];
		times = new double[numberOfSteps];
		setData();
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
	private Pose[] getCurvePoints(int numberOfSteps, Pose[] controlPoints) {
		Pose[] point2DList = new Pose[numberOfSteps + 1];

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
	private Pose getPoint(double percentage, Pose[] controlPoints) {
		double xCoordinateAtPercentage = 0;
		double yCoordinateAtPercentage = 0;

		int n = getDegree();

		for (int i = 0; i <= n; i++) {
			double coefficient = coefficients[i];

			double oneMinusT = Math.pow(1 - percentage, (double) (n - i));

			double powerOfT = Math.pow(percentage, (double) i);

			Pose pointI = controlPoints[i];

			xCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getX());
			yCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getY());
		}

		return new Pose(xCoordinateAtPercentage, yCoordinateAtPercentage, getAngle(percentage));
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
		if (t == 1) {
			dx = controlPoints[controlPoints.length - 1].getX() - controlPoints[controlPoints.length - 2].getX();
			dy = controlPoints[controlPoints.length - 1].getY() - controlPoints[controlPoints.length - 2].getY();
		}
		double angle = Math.atan2(dy, dx);
		return angle;
	}

	public void setData() {
		leftStates[0] = new State(lastPair.getLeft(), startVelocity, aMax);
		rightStates[0] = new State(lastPair.getRight(), startVelocity, aMax);
		times[0] = startTime;
		int n = centerPoses.length - 1;
		for (int i = 1; i < n; i++) {
			Object[] calculatedData = calculateData(i);
			leftStates[i] = (State) calculatedData[0];
			rightStates[i] = (State) calculatedData[1];
			times[i] = (double) calculatedData[2];
		}
	}

	private Object[] calculateData(int index) {
		Pose previousCenter = centerPoses[index - 1];
		Pose currentCenter = centerPoses[index];
		State previousLeft = leftStates[index - 1];
		State previousRight = rightStates[index - 1];
		double previousTime = times[index - 1];
		double previousLCenter = (previousLeft.getLength() + previousRight.getLength()) / 2;
		// When cruising, acceleration is 0
		double acceleration = 0;

		// The change in angle of the robot
		double dAngle = currentCenter.getAngle() - previousCenter.getAngle();

		// The change in distance of the robot sides
		double dLength = previousCenter.distance(currentCenter) * (isBackwards ? -1 : 1);
		double dlLeft = dLength - dAngle * robotWidth / 2;
		double dlRight = dLength + dAngle * robotWidth / 2;

		// The time required to get to the next point
		double dTime = Math.max(Math.abs(dlLeft), Math.abs(dlRight)) / vCruise;

		// The hypothetical velocity to get to that point
		double velocity = Math.abs(dLength) / dTime;

		// The average encoder distance to the next point
		double lCenter = previousLCenter + 0.5 * dLength - startLCenter;

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

		return new Object[] { new State(previousLeft.getLength() + dlLeft, velocityL, acceleration),
				new State(previousRight.getLength() + dlRight, velocityR, acceleration), previousTime + dTime };
	}

	@Override
	public Pose getStartingPosition() {
		return new Pose(centerPoses[0].getX(), centerPoses[0].getY(), Math.atan2(
				controlPoints[1].getY() - controlPoints[0].getY(), controlPoints[1].getX() - controlPoints[0].getX()));
	}

	@Override
	public PathData getPathData() {
		return new PathData(leftStates, rightStates, centerPoses, times);
	}
}
