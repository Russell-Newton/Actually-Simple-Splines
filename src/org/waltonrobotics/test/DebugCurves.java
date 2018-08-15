package org.waltonrobotics.test;

import java.util.ArrayList;
import java.util.List;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.dynamicMotion.DynamicBezierCurve;
import org.waltonrobotics.motion.BezierCurve;
import org.waltonrobotics.motion.Path;
import org.waltonrobotics.motion.PointTurn;
import org.waltonrobotics.motion.Spline;

/**
 * Run this class to see how changing points affects the curves
 *
 * @author Russell Newton, Walton Robotics
 */
public class DebugCurves {

	// Change these to see their effect
	private static double width = 0.70485;
	private static List<Pose> points = new ArrayList<>();
	private static boolean isBackwards = false;

	public static void main(String[] args) {
		testPaths();
//		testPathLengthTime();
	}

	private static void testPathLengthTime() {
		points.clear();
		points.add(new Pose(0, 0, StrictMath.toRadians(0)));
		points.add(new Pose(0, 1, StrictMath.toRadians(10)));

		for (int length = 0; length <= 10; length++) {
			Path.setPathNumberOfSteps((int) Math.pow(10, length));

			double pathNumber = Path.getPathNumberOfSteps();

			double startTime = System.currentTimeMillis();
			new PointTurn(1.0, 1, points.get(0), points.get(1).getAngle());
			System.out.println(pathNumber + ", Pointturn, " + (System.currentTimeMillis() - startTime));

			startTime = System.currentTimeMillis();
			new BezierCurve(1, 1, 0, 0, isBackwards, points);
			System.out.println(pathNumber + ", BezierCurve, " + (System.currentTimeMillis() - startTime));

			startTime = System.currentTimeMillis();
			new Spline(1, 1, 0, 0, isBackwards, points);
			System.out.println(pathNumber + ", Spline, " + (System.currentTimeMillis() - startTime));

			System.out.println(new Spline(1, 1, 0, 0, isBackwards, points).getPathData().getLast());

//			new Thread(() -> {
//				new PointTurn(1.0, 1, points.get(0), points.get(1).getAngle());
//				System.out.println(pathNumber + ": Pointturn: " + (System.currentTimeMillis() - startTime));
//			}).run();
//			new Thread(() -> {
//				new BezierCurve(1, 1, 0, 0, isBackwards, points);
//				System.out.println(pathNumber + ": BezierCurve: " + (System.currentTimeMillis() - startTime));
//			}).run();
//			new Thread(() -> {
//				new Spline(1, 1, 0, 0, isBackwards, points);
//				System.out.println(pathNumber + ": Spline: " + (System.currentTimeMillis() - startTime));
//			}).run();
		}
	}


	public static void printPath(Path path) {
		List<PathData> pathData = path.getPathData();

		for (PathData aPathData : pathData) {
			System.out.println(pathData);

			System.out.println(aPathData.getCenterPose().getX() + " " + aPathData.getCenterPose().getY() + ' '
				+ Math.toDegrees(aPathData.getCenterPose().getAngle()) + " t: " + aPathData.getTime());

			System.out.println(aPathData.getLeftState().getLength() + " " + aPathData.getRightState().getLength() + ' '
				+ aPathData.getLeftState().getVelocity() + ' ' + aPathData.getRightState().getVelocity());

			System.out.println();
		}
	}

	public static void testPaths() {
		points.clear();
		Path.setRobotWidth(width);
//		System.out.println("Point Turn:");
		PointTurn pointTurn = new PointTurn(1.0, 1, new Pose(0, 0, 0), StrictMath.toRadians(270));
//		printPath(pointTurn);

		points.add(new Pose(220, 40, StrictMath.toRadians(0)));
		points.add(new Pose(220, 260, StrictMath.toRadians(0)));
		points.add(new Pose(35, 200, StrictMath.toRadians(0)));
		points.add(new Pose(120, 160, StrictMath.toRadians(0)));

//		points.add(new Pose(35, 40, StrictMath.toRadians(0)));
//		points.add(new Pose(220, 40, StrictMath.toRadians(0)));
//		points.add(new Pose(0, 0, StrictMath.toRadians(0)));
//		points.add(new Pose(.5, 0, StrictMath.toRadians(0)));
//		points.add(new Pose(1, 1, StrictMath.toRadians(0)));
//		points.add(new Pose(2, 1, StrictMath.toRadians(0)));
//		points.add(new Pose(3.4, 5, StrictMath.toRadians(0)));
//		points.add(new Pose(1, 0, StrictMath.toRadians(0)));
//		points.add(new Pose(0, 1, StrictMath.toRadians(0)));
//		printPath(curve);
//
		System.out.println("Actual Length should be 272.87");

		DynamicBezierCurve dynamicBezierCurve = new DynamicBezierCurve(1, 1, 0, 0, isBackwards, points);

		for (int i = 0; i <= 100; i++) {
			Pose pose = dynamicBezierCurve.createPathData(new PathData(new Pose(0, 0)), i / 100.0).getCenterPose();

//			System.out.println(new Point2D(pose.getX(), pose.getY()).normalize());
		}

		System.out.println();

		System.out.println(dynamicBezierCurve.time);
//		System.out.println(points.get(0).distance(points.get(points.size() - 1)));
		System.out.println("Dynamic Bezier Curve:");

		System.out.println();
		System.out.println("Bezier Curve:");
//		BezierCurve curve = new BezierCurve(1, 1, 0, 0, isBackwards, points);
		PointTurn curve = new PointTurn(1, 1, points.get(0), .2);
//		System.out.println("Splitting: " + curve.curveLength);
		System.out.println(curve.getPathData().getLast().getTime());

		String curveString = curve.convertToString();
		System.out.println(curveString);

		Path path = Path.loadingPathFromString(curveString);
		System.out.println(path);

//		printPath(curve);

//		System.out.println("\n\nSpline:");
//		Spline spline = new Spline(100, 3, 0, 0, StrictMath.toRadians(0), StrictMath.toRadians(270), isBackwards,
//			points);
//		printPath(spline);

//		System.out.println("\n\nLine:");
//		Line line = new Line(1, 1, 0, 0, isBackwards, new Pose(0, 0, StrictMath.toRadians(0)),
//			new Pose(1, 1, StrictMath.toRadians(45)));
//		printPath(line);
	}

}
