package org.waltonrobotics.motion;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.State;
import org.waltonrobotics.util.GaussLegendre;

/**
 * Everything about Bezier Curves https://pomax.github.io/bezierinfo/ http://ttuadvancedrobotics.wikidot.com/trajectory-planning-for-point-to-point-motion
 */
public class DynamicBezierCurve extends DynamicPath {

	private final static HashMap<Integer, int[]> coefficents = new HashMap<>();
	private static HashMap<Key, GaussLegendre> gaussLegendreHashMap = new HashMap<>();

	static {
		coefficents.put(0, new int[]{1});
		coefficents.put(1, new int[]{1, 1});
		coefficents.put(2, new int[]{1, 2, 1});
		coefficents.put(3, new int[]{1, 3, 3, 1});
		coefficents.put(4, new int[]{1, 4, 6, 4, 1});
		coefficents.put(5, new int[]{1, 5, 10, 10, 5, 1});
		coefficents.put(6, new int[]{1, 6, 15, 20, 15, 6, 1});
		coefficents.put(7, new int[]{1, 7, 21, 35, 35, 21, 7, 1});
		coefficents.put(8, new int[]{1, 8, 28, 56, 70, 56, 28, 8, 1});
		coefficents.put(9, new int[]{1, 9, 36, 84, 126, 126, 84, 36, 9, 1});

		long start = System.nanoTime();
		calculateCoefficients(12);
		System.out.println((System.nanoTime() - start) / 1000000.0);
	}

	private final double startVelocity;
	private final double endVelocity;
	private final int degree;
	public double time;
	private int[] coefficients;
	private double curveLength;
	private double startLCenter;

	/**
	 * This constructor is used with the splines, but feel free to use it when creating your own motions
	 *
	 * @param vCruise - the cruise velocity of the robot
	 * @param aMax - the maximum acceleration of the robot
	 * @param startVelocity - the start velocity
	 * @param endVelocity - the end velocity
	 * @param isBackwards - whether or not to move the robot backwards
	 * @param controlPoints - the control points that define the curve
	 */
	public DynamicBezierCurve(double vCruise, double aMax, double startVelocity, double endVelocity,
		boolean isBackwards, List<Pose> controlPoints) {
		super(vCruise, aMax, isBackwards, controlPoints);
		this.startVelocity = startVelocity;
		this.endVelocity = endVelocity;
		// The starting average encoder distance should always be 0

		degree = getKeyPoints().size() - 1;
		coefficients = calculateCoefficients(degree);

		long startTime = System.nanoTime();
		computeArcLength(100, 0.001, 0.002);
		long endTime = System.nanoTime();
		System.out.println((endTime - startTime));
		System.out.println((endTime - startTime) / 1000000.0);
		System.out.println(curveLength);
		startTime = System.nanoTime();
		computeArcLengthSampling(10, 0., 0.002);
//		System.out.println();
		endTime = System.nanoTime();
		System.out.println(endTime - startTime);
		System.out.println((endTime - startTime) / 1000000.0);
		System.out.println(curveLength);
		time = computeTime();
	}

	public DynamicBezierCurve(double vCruise, double aMax, double startVelocity, double endVelocity,
		boolean isBackwards,
		Pose... controlPoints) {
		this(vCruise, aMax, startVelocity, endVelocity, isBackwards, Arrays.asList(controlPoints));
	}

	/**
	 * Uses the formula to find the value of nCr
	 *
	 * @return nCr
	 */
	private static long findNumberOfCombination(int n, int r) {
		int nFactorial = factorial(n);
		int rFactorial = factorial(r);
		int nMinusRFactorial = factorial(n - r);

		return nFactorial / (rFactorial * nMinusRFactorial);
	}

	/**
	 * Finds the factorial of any integer or double, d
	 *
	 * @return the factorial of d
	 */
	private static int factorial(int d) {
		if (d >= 13) {
			throw new ArithmeticException(String
				.format(
					"The number %d is too big of a number to factorize as it will cause integer overflow the maximum is 12",
					d));
		}

		int result = 1;
		try {

			for (int i = 1; i <= d; i++) {
				result = result * i;

			}
		} catch (ExceptionInInitializerError ignored) {

		}

		return result;
	}


	/**
	 * Updates the coefficients used for calculations
	 */
	private static int[] calculateCoefficients(int degree) {
		if (coefficents.containsKey(degree)) {
			return coefficents.get(degree);
		}

		int[] coefficients = new int[degree + 1];
		for (int i = 0; i < coefficients.length; i++) {
			coefficients[i] = Math.toIntExact(findNumberOfCombination(degree, i));
		}

		coefficents.put(degree, coefficients);

		return coefficients;
	}

	public double computeArcLengthSampling(double lower, double upper) {
		return computeArcLengthSampling(1000, lower, upper);
	}

	public double computeArcLengthSampling(int numberOfPoints, double lower, double upper) {

		double distance = 0;
		Pose previous = getPoint(lower);

		double increment = ((upper - lower) / numberOfPoints);
		for (int i = 1; i <= numberOfPoints; i++) {

			Pose point = getPoint((i * increment) + lower);
			distance += point.distance(previous);
			previous = point;
		}

		return distance;
	}

	private double computeTime() {
		double accelerationTime = calculateTime(startVelocity, getVCruise(), getAMax());
		double accelDistance = distance(startVelocity, getAMax(), accelerationTime);
		double decelerationTime = calculateTime(getVCruise(), endVelocity, -getAMax());
		double decelDistance = distance(getVCruise(), -getAMax(), accelerationTime);

		if (accelDistance + decelDistance > curveLength) {
			return calculateTime(startVelocity, endVelocity, getAMax());
		} else {
			double cruiseTime = (curveLength - (accelDistance + decelDistance)) / getVCruise();

			return accelerationTime + cruiseTime + decelerationTime;
		}
	}

	private double calculateTime(double startVelocity, double endVelocity, double acceleration) {
		return (endVelocity - startVelocity) / acceleration;
	}

	private double distance(double startVelocity, double constantAcceleration, double time) {
		return startVelocity * time + (constantAcceleration * time * time) / 2;
	}

	public double computeArcLength() {
		return computeArcLength(16, 0, 1);
	}

	public double computeArcLength(int n) {

		return computeArcLength(n, 0, 1);
	}

	public double computeArcLength(double lowerBound, double upperBound) {
		return computeArcLength(16, lowerBound, upperBound);
	}

	/**
	 * Uses the Gauss Legendre integration to approximate the arc length of the Bezier curve. This is the fastest
	 * technique (faster than sampling)
	 *
	 * @param n the number of integral strips
	 * @param lowerBound the lower bound to integrate (inclusive)
	 * @param upperBound the lower bound to integrate (inclusive)
	 * @return the arc length of the Bezier curve of t range of [lowerBound, upperBound]
	 */
	public double computeArcLength(int n, double lowerBound, double upperBound) {

		GaussLegendre gl;

		Key key = new Key(n, upperBound, lowerBound);
		if (gaussLegendreHashMap.containsKey(key)) {
			gl = gaussLegendreHashMap.get(key);
		} else {
			gl = new GaussLegendre(n, lowerBound, upperBound);
			gaussLegendreHashMap.put(key, gl);
		}

		double[] t = gl.getNodes();
		double[] C = gl.getWeights();

		double sum = 0;

		for (int i = 0; i < t.length; i++) {

			Pose point = getDerivative(t[i]);

			sum += C[i] * Math.hypot(point.getX(), point.getY());
		}

		return sum;
	}

	public double getStartVelocity() {
		return startVelocity;
	}

	public double getEndVelocity() {
		return endVelocity;
	}

	public int getDegree() {
		return degree;
	}

	public double getCurveLength() {
		return curveLength;
	}

	public double getTime() {
		return time;
	}

	@Override
	public PathData createPathData(PathData previousPathData, double percentage) {
		Pose centerPoint = getPoint(percentage);

		PathData pathData;
//		pathData= calculateData(startAverageEncoderLength, previousPathData, centerPoint);
		pathData = new PathData(centerPoint);
		return pathData;
	}

	private Pose getDerivative(double percentage) {
		double dx = 0;
		double dy = 0;

		if (percentage == 1.0) {

			int last = getKeyPoints().size() - 1;

			dx = getKeyPoints().get(last).getX()
				- getKeyPoints().get(last - 1).getX();
			dy = getKeyPoints().get(last).getY()
				- getKeyPoints().get(last - 1).getY();
		} else {

			int[] coefficients = calculateCoefficients(degree - 1);

			for (int i = 0; i < degree; i++) {

				Pose pointI = getKeyPoints().get(i);

				double multiplier =
					coefficients[i] * StrictMath.pow(1 - percentage, ((degree - 1) - i)) * StrictMath
						.pow(percentage, (double) i);

				Pose nextPointI = getKeyPoints().get(i + 1);

				dx += (multiplier = multiplier * (degree)) * (nextPointI.getX() - pointI.getX());
				dy += multiplier * (nextPointI.getY() - pointI.getY());
			}
		}

		double angle = StrictMath.atan2(dy, dx);

		if (isBackwards()) {
			angle += Math.PI;
		}
		angle %= (2 * Math.PI);

		return new Pose(dx, dy, angle);
	}

	private Pose getPoint(double percentage) {
		return getPoint(degree, percentage);
	}

	/**
	 * @param percentage - t
	 * @return the Pose that is at percentage t along the curve
	 */
	private Pose getPoint(int degree, double percentage) {
		double xCoordinateAtPercentage = 0;
		double yCoordinateAtPercentage = 0;

		double dx = 0;
		double dy = 0;

		int[] derivativeCoefficients = calculateCoefficients(degree - 1);

		for (int i = 0; i <= degree; i++) {

			Pose pointI = getKeyPoints().get(i);

			double multiplier = coefficients[i] *
				StrictMath.pow(1 - percentage, (degree - i)) *
				StrictMath.pow(percentage, (double) i);

			xCoordinateAtPercentage += (multiplier * pointI.getX());
			yCoordinateAtPercentage += (multiplier * pointI.getY());

			if (percentage != 1 && i < degree) {
				Pose nextPointI = getKeyPoints().get(i + 1);

				multiplier = derivativeCoefficients[i] *
					StrictMath.pow(1 - percentage, ((degree - 1) - i)) *
					StrictMath.pow(percentage, (double) i) * degree;

				dx += multiplier * (nextPointI.getX() - pointI.getX());
				dy += multiplier * (nextPointI.getY() - pointI.getY());
			}
		}

		if (percentage == 1.0) {
			int last = getKeyPoints().size() - 1;

			dx = getKeyPoints().get(last).getX()
				- getKeyPoints().get(last - 1).getX();
			dy = getKeyPoints().get(last).getY()
				- getKeyPoints().get(last - 1).getY();
		}

		double angle = StrictMath.atan2(dy, dx);

		if (isBackwards()) {
			angle += Math.PI;
		}
		angle %= (2 * Math.PI);

		return new Pose(xCoordinateAtPercentage, yCoordinateAtPercentage, angle);
	}

	public void setStartLCenter(double startLCenter) {
		this.startLCenter = startLCenter;
	}

	/**
	 * @param nextPosition - The Pose of the PathData to calculate on
	 * @return a new PathData from the calculations
	 */
	private PathData calculateData(PathData currentPathData, Pose nextPosition) {
		Pose previousCenter = currentPathData.getCenterPose();
		State previousLeft = currentPathData.getLeftState();
		State previousRight = currentPathData.getRightState();
		double previousTime = currentPathData.getTime();
		double previousLCenter = currentPathData.getLCenter();
		// When cruising, acceleration is 0
		double acceleration = 0;

		// The change in angle of the robot
		double dAngle = Path.boundAngle(nextPosition.getAngle() - previousCenter.getAngle());

		// The change in distance of the robot sides
		// FIXME This is probably wrong dLength should be 0 if there is not angle
//		double dLength = (previousCenter.sameCoordinates(nextPosition)? 0: previousCenter.distance(nextPosition)) * (isBackwards() ? -1 : 1);
		double dLength = previousCenter.distance(nextPosition) * (isBackwards() ? -1 : 1);
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
		Pose center = new Pose(nextPosition.getX(), nextPosition.getY(),
			previousCenter.getAngle() + dAngle);
		return new PathData(left, right, center, previousTime + dTime);
	}

	private class Key {

		int n;
		double upper;
		double lower;

		public Key(int n, double upper, double lower) {
			this.n = n;
			this.upper = upper;
			this.lower = lower;
		}
	}
}
