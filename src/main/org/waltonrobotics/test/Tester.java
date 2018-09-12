package org.waltonrobotics.test;

import java.util.ArrayList;
import java.util.List;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.dynamicMotion.DynamicBezierCurve;

public class Tester {

  private static List<Pose> points = new ArrayList<>();

  public static void main(String[] args) {
    points.add(new Pose(0, 0, StrictMath.toRadians(0)));
    points.add(new Pose(.5, 0, StrictMath.toRadians(0)));
    points.add(new Pose(1, 0, StrictMath.toRadians(0)));

//    Spline spline = new Spline(1, 1, 0, 0, false, points);
    DynamicBezierCurve spline = new DynamicBezierCurve(1, 1, 0, 0, false, points);
//    DynamicBezierCurve spline = new DynamicBezierCurve(1, 1, 0, 0, false, new Pose(0, 0), new Pose(1, 1));

    System.out.println(spline.getCurveLength());
    System.out.println(spline.getTime());
    System.out.println(spline.calculateData(.1));
//    System.out.println(spline.getPoint(0.0));
//    System.out.println(spline.getPoint(0.5));
//    System.out.println(spline.getDefiningBezierCurves().get(1).getPoint(0));
//    System.out.println(spline.getPoint(1.0));
//    System.out.println(spline.getPoint(0.0));
  }


}

