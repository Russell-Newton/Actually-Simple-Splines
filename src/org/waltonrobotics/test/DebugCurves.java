package org.waltonrobotics.test;

import java.util.ArrayList;
import java.util.List;
import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.BezierCurve;
import org.waltonrobotics.motion.Line;
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

		Path.setRobotWidth(width);
		PointTurn pointTurn = new PointTurn(1.0, 1, new Pose(0, 0, 0), StrictMath.toRadians(270));
		System.out.println("Point Turn:");
		printPath(pointTurn);

		points.add(new Pose(0, 0, StrictMath.toRadians(0)));
		points.add(new Pose(1, 1, StrictMath.toRadians(90)));
		BezierCurve curve = new BezierCurve(1, 1, 0, 0, isBackwards, points);
		System.out.println("Bezier Curve:");
		printPath(curve);

		Spline spline = new Spline(100, 3, 0, 0, StrictMath.toRadians(0), StrictMath.toRadians(270), isBackwards,
			points);
		System.out.println("\n\nSpline:");
		printPath(spline);

		Line line = new Line(1, 1, 0, 0, isBackwards, new Pose(0, 0, StrictMath.toRadians(0)),
			new Pose(1, 1, StrictMath.toRadians(45)));
		System.out.println("\n\nLine:");
		printPath(line);
	}

	public static void printPath(Path path) {
		List<PathData> pathData = path.getPathData();

		for (PathData aPathData : pathData) {
			System.out.println(aPathData.getCenterPose().getX() + " " + aPathData.getCenterPose().getY() + ' '
				+ Math.toDegrees(aPathData.getCenterPose().getAngle()) + " t: " + aPathData.getTime());

			System.out.println(aPathData.getLeftState().getLength() + " " + aPathData.getRightState().getLength() + ' '
				+ aPathData.getLeftState().getVelocity() + ' ' + aPathData.getRightState().getVelocity());

			System.out.println();
		}
	}
}
