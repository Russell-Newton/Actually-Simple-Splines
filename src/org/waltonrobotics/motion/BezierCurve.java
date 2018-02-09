package org.waltonrobotics.motion;

import java.util.ArrayList;
import java.util.List;

import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
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

	private final List<PathData> pathData;

	private final List<Pose> controlPoints;
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
			PathData startPathData, List<Pose> controlPoints) {
		super(vCruise, aMax, robotWidth);
		this.controlPoints = controlPoints;
		this.numberOfSteps = 50;
		this.isBackwards = isBackwards;
		startVelocity = v0;
		endVelocity = v1;
		// The starting average encoder distance should always be 0
		startLCenter = startPathData.getLCenter();
		updateCoefficients();
		pathData = new ArrayList<>(numberOfSteps);
		getCurveLength();
		setData(startPathData);
	}

	public BezierCurve(double vCruise, double aMax, double v0, double v1, double robotWidth, boolean isBackwards,
			List<Pose> controlPoints) {
		this(vCruise, aMax, v0, v1, robotWidth, isBackwards,
				new PathData(new Pose(controlPoints.get(0).getX(), controlPoints.get(0).getY(),
						Math.atan2(controlPoints.get(1).getY() - controlPoints.get(0).getY(),
								controlPoints.get(1).getX() - controlPoints.get(0).getX()))),
				controlPoints);
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

	private void getCurveLength() {
		curveLength = 0;
		for (double i = 1; i < numberOfSteps; i++) {
			curveLength += getPoint(i / numberOfSteps).distance(getPoint((i - 1) / numberOfSteps));
		}
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
	private Pose getPoint(double percentage) {
		double xCoordinateAtPercentage = 0;
		double yCoordinateAtPercentage = 0;

		int n = getDegree();

		for (int i = 0; i <= n; i++) {
			double coefficient = coefficients[i];

			double oneMinusT = Math.pow(1 - percentage, (double) (n - i));

			double powerOfT = Math.pow(percentage, (double) i);

			Pose pointI = controlPoints.get(i);

			xCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getX());
			yCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getY());
		}

		return new Pose(xCoordinateAtPercentage, yCoordinateAtPercentage, getAngle(percentage));
	}

	/**
	 * @return the degree of the curve
	 */
	private int getDegree() {
		return controlPoints.size() - 1;
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
			dx += coefficient * (n + 1) * (controlPoints.get(i + 1).getX() - controlPoints.get(i).getX());
			dy += coefficient * (n + 1) * (controlPoints.get(i + 1).getY() - controlPoints.get(i).getY());
		}
		if (t == 1) {
			dx = controlPoints.get(controlPoints.size() - 1).getX()
					- controlPoints.get(controlPoints.size() - 2).getX();
			dy = controlPoints.get(controlPoints.size() - 1).getY()
					- controlPoints.get(controlPoints.size() - 2).getY();
		}
		double angle = Math.atan2(dy, dx);
		return angle;
	}

	public void setData(PathData startData) {
		PathData previousData = startData;
		PathData currentData;
		for (int i = 1; i <= numberOfSteps; i++) {
			currentData = calculateData(previousData, getPoint((double) i / numberOfSteps));
			pathData.add(currentData);
			previousData = currentData;
		}
	}

	private PathData calculateData(PathData previousPathData, Pose currentCenter) {
		Pose previousCenter = previousPathData.getCenterPose();
		State previousLeft = previousPathData.getLeftState();
		State previousRight = previousPathData.getRightState();
		double previousTime = previousPathData.getTime();
		double previousLCenter = previousPathData.getLCenter();
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
		// System.out.println("acc: " + vAccelerating + " dec: " + vDecelerating + "
		// vel: " + velocity);
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

		State left = new State(previousLeft.getLength() + dlLeft, velocityL, acceleration);
		State right = new State(previousRight.getLength() + dlRight, velocityR, acceleration);
		return new PathData(left, right, currentCenter, previousTime + dTime);
	}

	@Override
	public List<PathData> getPathData() {
		return pathData;
	}
}
