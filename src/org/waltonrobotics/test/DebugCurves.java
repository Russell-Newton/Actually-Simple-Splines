package org.waltonrobotics.test;

import java.util.ArrayList;
import java.util.List;
import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.BezierCurve;
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
	private static boolean isBackwards;

	public static void main(String[] args) {
		points.add(new Pose(0, 0));
		points.add(new Pose(0, 5.18));
		points.add(new Pose(5.64, 5.18));
		points.add(new Pose(5.64, 1));
		points.add(new Pose(0.3, 1));
		BezierCurve curve = new BezierCurve(1, 1, 0, 0, isBackwards,
			points
		);
		System.out.println("Bezier Curve:");
		printPath(curve);

		Spline spline = new Spline(2, 2, 0, 0, 90, 180, isBackwards, points);
		System.out.println("\n\nSpline:");
		printPath(spline);
	}

	public static void printPath(Path path) {
		List<PathData> pathData = path.getPathData();

		for (PathData aPathData : pathData) {
			System.out.println(
				aPathData.getCenterPose().getX() + " " + aPathData.getCenterPose()
					.getY()
					+ ' ' + Math.toDegrees(aPathData.getCenterPose().getAngle()) + " t: "
					+ aPathData.getTime());

			System.out.println(aPathData.getLeftState().getLength() + " "
				+ aPathData.getRightState().getLength() + ' ' + aPathData.getLeftState()
				.getVelocity()
				+ ' ' + aPathData.getRightState().getVelocity());

			System.out.println();
		}
	}
}
