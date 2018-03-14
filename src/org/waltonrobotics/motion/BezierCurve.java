package org.waltonrobotics.motion;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.State;

/**
 * <p>This Path is a simple curve. The shape of the curve is controlled by the control points. There
 * are tangents from the first and second control points and the second to last and last control
 * points. Use this to make a straight line (use two control points) <br> <a
 * href=https://en.wikipedia.org/wiki/B%C3%A9zier_curve>Wikipedia page on Bezier Curves</a> <br> <a
 * href=https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html>Bezier curve
 * equations for N number of control points</a>
 * <p/>
 *
 * @author Marius Juston, Walton Robotics
 * @author Russell Newton, Walton Robotics
 */
public class BezierCurve extends Path {

	private final double startVelocity;
	private final double endVelocity;
	private final double startLCenter;
	private final int numberOfSteps;
	private final LinkedList<PathData> pathData;
	private double curveLength;
	private double[] coefficients;

	/**
	 * This constructor is used with the splines, but feel free to use it when creating your own
	 * motions
	 *
	 * @param vCruise - the cruise velocity of the robot
	 * @param aMax - the maximum acceleration of the robot
	 * @param v0 - the start velocity
	 * @param v1 - the end velocity
	 * @param isBackwards - whether or not to move the robot backwards
	 * @param startPathData - the starting PathData for the curve
	 * @param controlPoints - the control points that define the curve
	 */
	public BezierCurve(double vCruise, double aMax, double v0, double v1,
		boolean isBackwards,
		PathData startPathData, List<Pose> controlPoints) {
		super(vCruise, aMax, isBackwards, controlPoints);
		numberOfSteps = 50;
		startVelocity = v0;
		endVelocity = v1;
		// The starting average encoder distance should always be 0
		startLCenter = startPathData.getLCenter();
		updateCoefficients();
		pathData = new LinkedList<>();
		getCurveLength();
		setData(startPathData);
	}

	/**
	 * Use this if you don't need to define a starting PathData
	 */
	public BezierCurve(double vCruise, double aMax, double v0, double v1,
		boolean isBackwards,
		List<Pose> controlPoints) {
		this(vCruise, aMax, v0, v1, isBackwards,
			(controlPoints.size() == 0) ?
				new PathData(new Pose(0, 0)) :
				(controlPoints.size() == 1) ?
					new PathData(
						new Pose(
							controlPoints.get(0).getX(),
							controlPoints.get(0).getY(),
							controlPoints.get(0).getAngle())
					) :
					new PathData(
						new Pose(
							controlPoints.get(0).getX(),
							controlPoints.get(0).getY(),
							StrictMath.atan2(
								controlPoints.get(1).getY() - controlPoints.get(0).getY(),
								controlPoints.get(1).getX() - controlPoints.get(0).getX())
						)
					),
			controlPoints);
	}

	public BezierCurve(double vCruise, double aMax, double v0, double v1,
		boolean isBackwards,
		Pose... controlPoints) {
		this(vCruise, aMax, v0, v1, isBackwards, Arrays.asList(controlPoints));
	}

	/**
	 * Uses the formula to find the value of nCr
	 *
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
	 * @return the factorial of d
	 */
	private static double factorial(double d) {
		double r = (d - Math.floor(d)) + 1;
		while (d > 1) {
			r *= d;
			d -= 1;
		}
		return r;
	}

	/**
	 * Caluclates the length of the curve
	 */
	private void getCurveLength() {
		curveLength = 0;

		if (getKeyPoints().size() > 1) {
			for (double i = 1; i < numberOfSteps; i++) {
				curveLength += getPoint(i / numberOfSteps)
					.distance(getPoint((i - 1) / numberOfSteps));
			}
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
	 * @param percentage - t
	 * @return the Pose that is at percentage t along the curve
	 */
	private Pose getPoint(double percentage) {
		double xCoordinateAtPercentage = 0;
		double yCoordinateAtPercentage = 0;

		int n = getDegree();

		for (int i = 0; i <= n; i++) {
			double coefficient = coefficients[i];

			double oneMinusT = StrictMath.pow(1 - percentage, (double) (n - i));

			double powerOfT = StrictMath.pow(percentage, (double) i);

			Pose pointI = getKeyPoints().get(i);

			xCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getX());
			yCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getY());
		}

		return new Pose(xCoordinateAtPercentage, yCoordinateAtPercentage, getAngle(percentage));
	}

	/**
	 * @return the degree of the curve
	 */
	private int getDegree() {
		return getKeyPoints().size() - 1;
	}

	/**
	 * @param t - percent along curve
	 * @return angle at point
	 */
	private double getAngle(double t) {
		int n = getDegree();
		double dx = 0;
		double dy = 0;
		for (int i = 0; i < n; i++) {
			double coefficient =
				findNumberOfCombination(n, i) * StrictMath.pow(t, i) * StrictMath.pow(1 - t, n - i);
			dx += coefficient * (n + 1) * (getKeyPoints().get(i + 1).getX() - getKeyPoints().get(i)
				.getX());
			dy += coefficient * (n + 1) * (getKeyPoints().get(i + 1).getY() - getKeyPoints().get(i)
				.getY());
		}
		if (t == 1.0) {
			dx = getKeyPoints().get(getKeyPoints().size() - 1).getX()
				- getKeyPoints().get(getKeyPoints().size() - 2).getX();
			dy = getKeyPoints().get(getKeyPoints().size() - 1).getY()
				- getKeyPoints().get(getKeyPoints().size() - 2).getY();
		}
		double angle = StrictMath.atan2(dy, dx);
		if (isBackwards()) {
			angle += Math.PI;
		}
		angle %= (2 * Math.PI);
		return angle;
	}

	/**
	 * Creates the PathData list
	 */
	private void setData(PathData data) {
		for (int i = 1; i <= numberOfSteps; i++) {
			data = calculateData(data, getPoint((double) i / numberOfSteps));
			pathData.add(data);
		}
	}

	/**
	 * @param currentCenter - The Pose of the PathData to calculate on
	 * @return a new PathData from the calculations
	 */
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
		if (dAngle > Math.PI) {
			dAngle -= 2 * Math.PI;
		} else if (dAngle < -Math.PI) {
			dAngle += 2 * Math.PI;
		}

		// The change in distance of the robot sides
		double dLength = previousCenter.distance(currentCenter) * (isBackwards() ? -1 : 1);
		double dlLeft = dLength - ((dAngle * getRobotWidth()) / 2);
		double dlRight = dLength + ((dAngle * getRobotWidth()) / 2);

		// The time required to get to the next point
		double dTime = Math.max(Math.abs(dlLeft), Math.abs(dlRight)) / getVCruise();

		// The hypothetical velocity to get to that point
		double velocity = dLength / dTime;

		// The average encoder distance to the next point
		double lCenter = (previousLCenter + (0.5 * dLength)) - startLCenter;
		double vAccelerating;
		double vDecelerating;

		if (isBackwards()) {
			vAccelerating = -Math
				.sqrt(StrictMath.pow(startVelocity, 2) + (getAMax() * Math.abs(lCenter)));
			vDecelerating = -Math
				.sqrt(
					StrictMath.pow(endVelocity, 2) + (getAMax() * Math
						.abs(curveLength - Math.abs(lCenter))));
			if ((vAccelerating > velocity) && (vAccelerating > vDecelerating)) {
				acceleration = -getAMax();
				dTime = dLength / vAccelerating;
			}
			if ((vDecelerating > velocity) && (vDecelerating > vAccelerating)) {
				acceleration = getAMax();
				dTime = dLength / vDecelerating;
			}
		} else {
			vAccelerating = Math
				.sqrt(StrictMath.pow(startVelocity, 2) + (getAMax() * Math.abs(lCenter)));
			vDecelerating = Math
				.sqrt(
					StrictMath.pow(endVelocity, 2) + (getAMax() * Math
						.abs(curveLength - Math.abs(lCenter))));

			if ((vAccelerating < velocity) && (vAccelerating < vDecelerating)) {
				acceleration = getAMax();
				dTime = dLength / vAccelerating;
			}
			if ((vDecelerating < velocity) && (vDecelerating < vAccelerating)) {
				acceleration = -getAMax();
				dTime = dLength / vDecelerating;
			}
		}
//		System.out
//			.println("acc: " + vAccelerating + " dec: " + vDecelerating + " vel: " + velocity + " velAct: " + dLength/dTime);
		double velocityL = dlLeft / dTime;
		double velocityR = dlRight / dTime;

		State left = new State(previousLeft.getLength() + dlLeft, velocityL, acceleration);
		State right = new State(previousRight.getLength() + dlRight, velocityR, acceleration);
		Pose center = new Pose(currentCenter.getX(), currentCenter.getY(),
			previousCenter.getAngle() + dAngle);
		return new PathData(left, right, center, previousTime + dTime);
	}

	@Override
	public final LinkedList<PathData> getPathData() {
		return pathData;
	}

	@Override
	public final String toString() {
		return "BezierCurve{" +
			"startVelocity=" + startVelocity +
			", endVelocity=" + endVelocity +
			", startLCenter=" + startLCenter +
			", curveLength=" + curveLength +
			", numberOfSteps=" + numberOfSteps +
			", isBackwards=" + isBackwards() +
			", pathData=" + pathData +
			", controlPoints=" + getKeyPoints() +
			", coefficients=" + Arrays.toString(coefficients) +
			"} " + super.toString();
	}
}
