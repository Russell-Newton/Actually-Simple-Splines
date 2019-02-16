package org.waltonrobotics.test;

//import java.net.BindException;
//import java.util.ArrayList;
//import java.util.List;

import static org.waltonrobotics.motion.Path.getPathNumberOfSteps;

import java.util.ArrayList;
import java.util.List;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.dynamicMotion.DynamicBezierCurve;
import org.waltonrobotics.dynamicMotion.DynamicSpline;
import org.waltonrobotics.motion.BezierCurve;
import org.waltonrobotics.motion.Spline;

public class Tester {

  private static List<Pose> points = new ArrayList<>();


  public static void main(String[] args) {

//    points.add(new Pose(0, 0, StrictMath.toRadians(0)));
    points.add(new Pose(0, 0, StrictMath.toRadians(0)));
//    points.add(new Pose(.5, 0, StrictMath.toRadians(0)));
    points.add(new Pose(1.0, 0, StrictMath.toRadians(0)));
//    points.add(new Pose(1, 1, StrictMath.toRadians(0)));
//    points.add(new Pose(1.5, 0, StrictMath.toRadians(0)));
//    points.add(new Pose(2, 0, StrictMath.toRadians(0)));
//    points.add(new Pose(2, 2, StrictMath.toRadians(0)));

//    Spline spline = new Spline(1, 1, 0, 0, false, points);
    {
      BezierCurve spline = new BezierCurve(1.0, 1.0, 0, 0, false, points);
      DynamicBezierCurve dynamicBezierCurve = new DynamicBezierCurve(1.0, 1.0, 0, 0, false, points);

      System.out.println(spline.getCurveLength());
      System.out.println(dynamicBezierCurve.getCurveLength());

      System.out.println("============================= BEZIER CURVE ===========================================");
      System.out.println("----------------------------- POSE -------------------------------------------");
      for (double i = 0; i <= getPathNumberOfSteps(); i++) {
        System.out.println(spline.getPoint(i / getPathNumberOfSteps()));
        System.out.println(dynamicBezierCurve.getPoint(i / getPathNumberOfSteps()));

      }
      System.out.println();

      System.out.println("----------------------------- DERIVATIVES -------------------------------------------");
      for (double i = 0; i <= getPathNumberOfSteps(); i++) {

        System.out.println(spline.getDerivative(i / getPathNumberOfSteps()));
        System.out.println(dynamicBezierCurve.getDerivative(i / getPathNumberOfSteps()));
      }
      System.out.println();
    }

    {
      Spline spline = new Spline(1.0, 1.0, 0, 0, false, points);
      DynamicSpline dynamicBezierCurve = new DynamicSpline(1.0, 1.0, 0, 0, false, points);

      BezierCurve bezier = new BezierCurve(1.0, 1.0, 0, 0, false, points);
      DynamicBezierCurve dynamic = new DynamicBezierCurve(1.0, 1.0, 0, 0, false, points);

      System.out.println(spline.getCurveLength());
//      System.out.println(dynamicBezierCurve.getCurveLength());

      System.out.println("============================= SPLINES ===========================================");
      System.out.println("----------------------------- POSE -------------------------------------------");
      for (double i = 0; i <= getPathNumberOfSteps(); i++) {
        double p = i / getPathNumberOfSteps();
        System.out.println(p);

        System.out.println(bezier.getPoint(p));
        System.out.println(spline.getPoint(p));

        System.out.println(dynamic.getPoint(p));
        System.out.println(dynamicBezierCurve.getPoint(p));
      }
      System.out.println();

      System.out.println("----------------------------- DERIVATIVE -------------------------------------------");
      for (double i = 0; i <= getPathNumberOfSteps(); i++) {
        double p = i / getPathNumberOfSteps();
        System.out.println(p);

//        System.out.println(bezier.getDerivative(p));
        System.out.println(spline.getDerivative(p));

//        System.out.println(dynamic.getDerivative(p));
        System.out.println(dynamicBezierCurve.getDerivative(p));
      }
      System.out.println();

      System.out.println(spline.getCurveLength());
      System.out.println(dynamicBezierCurve.getCurveLength());
    }

//    System.out.println(spline.getCurveLength());
//    System.out.println(spline.getTime());
//    System.out.println(spline.calculateData(.1));
//    System.out.println(spline.getPoint(0.0));
//    System.out.println(spline.getPoint(0.5));
//    System.out.println(spline.getDefiningBezierCurves().get(1).getPoint(0));
//    System.out.println(spline.getPoint(1.0));
//    System.out.println(spline.getPoint(0.0));
  }


}

