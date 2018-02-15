package org.waltonrobotics;

import java.util.ArrayList;
import java.util.List;

import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.BezierCurve;
import org.waltonrobotics.motion.Spline;

/**
 * Run this class to see how changing points affects the curves
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class DebugCurves {

	// Change these to see their effect
	private static double width = .70485;
	private static List<Pose> points = new ArrayList<Pose>();
	private static boolean isBackwards = false;

	public static void main(String[] args) {
		points.add(new Pose(0, 0));
		points.add(new Pose(0, 5.18));
		points.add(new Pose(5.64, 5.18));
		points.add(new Pose(5.64, 1));
		points.add(new Pose(.3, 1));

		System.out.println("Bezier Curve:");
		BezierCurve curve = new BezierCurve(1, 1, 0, 0, width, isBackwards, points);
		List<PathData> pathData = curve.getPathData();
		for (int i = 0; i < pathData.size(); i++) {
			System.out.println(pathData.get(i).getCenterPose().getX() + " " + pathData.get(i).getCenterPose().getY()
					+ " " + Math.toDegrees(pathData.get(i).getCenterPose().getAngle()) + " t: "
					+ pathData.get(i).getTime());
		}
		System.out.println("\n \n Spline:");
		Spline spline = new Spline(2, 2, 0, 0, width, 90, 180, isBackwards, points);
		pathData = spline.getPathData();
		for (int i = 0; i < pathData.size(); i++) {
			System.out.println(pathData.get(i).getCenterPose().getX() + " " + pathData.get(i).getCenterPose().getY()
					+ " " + Math.toDegrees(pathData.get(i).getCenterPose().getAngle()) + " t: "
					+ pathData.get(i).getTime());
			System.out.println(pathData.get(i).getLeftState().getLength() + " "
					+ pathData.get(i).getRightState().getLength() + " " + pathData.get(i).getLeftState().getVelocity()
					+ " " + pathData.get(i).getRightState().getVelocity() + "\n");
		}
	}
}
