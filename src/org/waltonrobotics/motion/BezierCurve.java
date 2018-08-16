package org.waltonrobotics.motion;

import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.State;
import org.waltonrobotics.util.GaussLegendre;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

/**
 * <p>This Path is a simple curve. The shape of the curve is controlled by the control points.
 * There are tangents from the first and second control points and the second to last and last
 * control points. Use this to make a straight line (use two control points) <br> <a
 * href=https://en.wikipedia.org/wiki/B%C3%A9zier_curve>Wikipedia page on Bezier Curves</a> <br> <a
 * href=https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html>Bezier curve
 * equations for N number of control points</a>
 * <p/>
 *
 * @author Marius Juston, Walton Robotics
 * @author Russell Newton, Walton Robotics
 */
public class BezierCurve extends Path {

    public final static HashMap<Integer, int[]> coefficents = new HashMap<>();
    public static HashMap<Key, GaussLegendre> gaussLegendreHashMap = new HashMap<>();

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
    }


    private final double startVelocity;
    private final double endVelocity;
    private final PathData startPathData;
    private final double startLCenter;
    public double curveLength;
    private List<Pose> pathPoints;
    private int[] coefficients;

    /**
     * This constructor is used with the splines, but feel free to use it when creating your own
     * motions
     *
     * @param vCruise       - the cruise velocity of the robot
     * @param aMax          - the maximum acceleration of the robot
     * @param startVelocity - the start velocity
     * @param endVelocity   - the end velocity
     * @param isBackwards   - whether or not to move the robot backwards
     * @param startPathData - the starting PathData for the curve
     * @param controlPoints - the control points that define the curve
     */
    public BezierCurve(double vCruise, double aMax, double startVelocity, double endVelocity,
                       boolean isBackwards,
                       PathData startPathData, List<Pose> controlPoints) {
        super(vCruise, aMax, isBackwards, controlPoints);
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.startPathData = startPathData;
        // The starting average encoder distance should always be 0
        startLCenter = startPathData.getLCenter();

        createPath();
    }

    /**
     * Use this if you don't need to define a starting PathData
     */
    public BezierCurve(double vCruise, double aMax, double startVelocity, double endVelocity,
                       boolean isBackwards,
                       List<Pose> controlPoints) {
        this(vCruise, aMax, startVelocity, endVelocity, isBackwards,
                (controlPoints.isEmpty()) ?
                        new PathData(new Pose(0, 0), isBackwards) :
                        ((controlPoints.size() == 1) ?
                                new PathData(
                                        new Pose(
                                                controlPoints.get(0).getX(),
                                                controlPoints.get(0).getY(),
                                                controlPoints.get(0).getAngle())
                                        , isBackwards) :
                                new PathData(
                                        new Pose(
                                                controlPoints.get(0).getX(),
                                                controlPoints.get(0).getY(),
                                                StrictMath.atan2(
                                                        controlPoints.get(1).getY() - controlPoints.get(0).getY(),
                                                        controlPoints.get(1).getX() - controlPoints.get(0).getX())
                                        )
                                        , isBackwards)),
                controlPoints);
    }

    public BezierCurve(double vCruise, double aMax, double startVelocity, double endVelocity,
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
     * Finds the factorial of any integer d
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
     * Calculates the binomial coefficients for the demanded path degree
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

    private List<Pose> createPoints() {
        List<Pose> pathPoints = new LinkedList<>();

        for (double i = 0; i <= getPathNumberOfSteps(); i++) {

            pathPoints.add(getPoint(i / getPathNumberOfSteps()));
        }

        return pathPoints;
    }

    /**
     * Caluclates the length of the curve
     */
    private double getCurveLength() {
        double curveLength = 0;

        if (getKeyPoints().size() > 1) {
            for (int i = 1; i < getPathNumberOfSteps(); i++) {
                curveLength += pathPoints.get(i).distance(pathPoints.get(i - 1));
            }
        }

        return curveLength;
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

            double oneMinusT = StrictMath.pow(1 - percentage, (n - i));

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
        Pose getDerivative = getDerivative(t);

        return getDerivative.getAngle();
    }

    /**
     * Creates the PathData list
     */
    private void setData(PathData data) {

        for (int i = 1; i <= getPathNumberOfSteps(); i++) {
            data = calculateData(data, pathPoints.get(i));
            getPathData().add(data);
        }

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

    public void createPath() {
        coefficients = calculateCoefficients(getDegree());
        pathPoints = createPoints();
        curveLength = getCurveLength();
        setData(startPathData);
    }

    public double computeArcLength() {
        return computeArcLength(16 /* this number seems to give decent accuracy*/, 0, 1);
    }

    public double computeArcLength(int n) {

        return computeArcLength(n, 0, 1);
    }

    public double computeArcLength(double lowerBound, double upperBound) {
        return computeArcLength(16, lowerBound, upperBound);
    }

    /**
     * Uses the Gauss Legendre integration to approximate the arc length of the Bezier curve. This is
     * the fastest technique (faster than sampling) when having a large path and shows the most
     * accurate results
     *
     * @param n          the number of integral strips 2+ more means better accuracy
     * @param lowerBound the lower bound to integrate (inclusive) [0,1]
     * @param upperBound the upper bound to integrate (inclusive) [0,1]
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

    /**
     * Gets the derivative of point at value percentage
     */
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

            int degree = getDegree();

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


    public String convertToString() {
			/*
		double vCruise,
		double aMax,
		double startVelocity,
		double endVelocity,
		boolean isBackwards,
		List<Pose> controlPoints
		 */

        String className = getClass().getName();

        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(className);
        stringBuilder.append(' ');
        stringBuilder.append(getVCruise());
        stringBuilder.append(' ');
        stringBuilder.append(getAMax());
        stringBuilder.append(' ');
        stringBuilder.append(startVelocity);
        stringBuilder.append(' ');
        stringBuilder.append(endVelocity);
        stringBuilder.append(' ');
        stringBuilder.append(isBackwards());
        stringBuilder.append(' ');

        addKeyPoints(stringBuilder);
        return stringBuilder.toString();
    }


    @Override
    public String toString() {
        return "BezierCurve{" +
                "startVelocity=" + startVelocity +
                ", endVelocity=" + endVelocity +
                ", startPathData=" + startPathData +
                ", startLCenter=" + startLCenter +
                ", pathPoints=" + pathPoints +
                ", curveLength=" + curveLength +
                ", coefficients=" + Arrays.toString(coefficients) +
                "} " + super.toString();
    }

    public double getStartVelocity() {
        return startVelocity;
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    public static class Key {

        int n;
        double upper;
        double lower;

        public Key(int n, double upper, double lower) {
            this.n = n;
            this.upper = upper;
            this.lower = lower;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) {
                return true;
            }
            if (o == null || getClass() != o.getClass()) {
                return false;
            }

            Key key = (Key) o;

            if (n != key.n) {
                return false;
            }
            if (Double.compare(key.upper, upper) != 0) {
                return false;
            }
            return Double.compare(key.lower, lower) == 0;
        }

        @Override
        public int hashCode() {
            int result;
            long temp;
            result = n;
            temp = Double.doubleToLongBits(upper);
            result = 31 * result + (int) (temp ^ (temp >>> 32));
            temp = Double.doubleToLongBits(lower);
            result = 31 * result + (int) (temp ^ (temp >>> 32));
            return result;
        }
    }
}
