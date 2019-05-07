package org.waltonrobotics.motion.swerve;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import org.waltonrobotics.metadata.KinematicParameters;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.State;
import org.waltonrobotics.metadata.swerve.SwervePathData;
import org.waltonrobotics.metadata.swerve.SwerveState;

/**
 * @author Russell Newton
 **/
public class SwerveSpline extends SwervePath {

  private final Double startAngle;
  private final Double endAngle;
  private final double startScale;
  private final double endScale;
  private final boolean useKnotAngles;

  private List<SwerveBezierCurve> definingBezierCurves = new ArrayList<>();

  /**
   * Creates a spline for a Swerve-Drive robot
   * @param kinematicParameters - the maximum translational velocity and acceleration
   * @param startSwerveState - the state of the robot at the start
   * @param endSwerveState - the end state of the robot
   * @param startAngle - the
   * @param endAngle
   * @param startScale
   * @param endScale
   * @param useKnotAngles
   * @param knots
   */
  public SwerveSpline(KinematicParameters kinematicParameters, SwerveState startSwerveState,
      SwerveState endSwerveState, Double startAngle, Double endAngle,
      double startScale, double endScale, boolean useKnotAngles,
      List<Pose> knots) {
    super(kinematicParameters, startSwerveState, endSwerveState, knots);
    this.startAngle = startAngle;
    this.endAngle = endAngle;
    this.startScale = startScale;
    this.endScale = endScale;
    this.useKnotAngles = useKnotAngles;

    createPath();
  }

  public SwerveSpline(KinematicParameters kinematicParameters, SwerveState startSwerveState,
      SwerveState endSwerveState, Double startAngle, Double endAngle, List<Pose> knots) {
    this(kinematicParameters, startSwerveState, endSwerveState, startAngle, endAngle, 1, 1, true, knots);
  }

  public SwerveSpline(KinematicParameters kinematicParameters, SwerveState startSwerveState,
      SwerveState endSwerveState,
      List<Pose> knots) {
    this(kinematicParameters, startSwerveState, endSwerveState, null, null, 1, 1, true, knots);
  }

  private void createPath() {
    List<List<Pose>> pathControlPoints = computeControlPoints(getKeyPoints());

//		System.out.println(pathControlPoints.size());

    SwervePathData startPathData = new SwervePathData(new SwerveState(),
        new Pose(pathControlPoints.get(0).get(0).getX(), pathControlPoints.get(0).get(0).getY(),
            startSwerveState.getCenterAngle()),
        0);

    definingBezierCurves = createBezierCurves(startPathData, pathControlPoints);
//		System.out.println(bezierCurves.size());

    stitchPathData();
  }

  /**
   * Creates the control points required to make cubic bezier curves that transition between knots. Will make them for
   * the shortest path possible.
   *
   * @return A list of lists that hold the control points for the segments in the spline
   */
  private List<List<Pose>> computeControlPoints(List<Pose> knots) {
    int degree = knots.size() - 1;
    Pose[] points1 = new Pose[degree];
    Pose[] points2 = new Pose[degree];

    /* constants for Thomas Algorithm */
    double[] a = new double[degree];
    double[] b = new double[degree];
    double[] c = new double[degree];
    double[] rX = new double[degree];
    double[] rY = new double[degree];

    /* left most segment */
    a[0] = 0;
    b[0] = 2.0;
    c[0] = 1.0;
    rX[0] = knots.get(0).getX() + (2.0 * knots.get(1).getX());
    rY[0] = knots.get(0).getY() + (2.0 * knots.get(1).getY());

    /* internal segments */
    for (int i = 1; i < (degree - 1); i++) {
      a[i] = 1.0;
      b[i] = 4.0;
      c[i] = 1.0;
      rX[i] = (4.0 * knots.get(i).getX()) + (2.0 * knots.get(i + 1).getX());
      rY[i] = (4.0 * knots.get(i).getY()) + (2.0 * knots.get(i + 1).getY());
    }

    /* right segment */
    a[degree - 1] = 2.0;
    b[degree - 1] = 7.0;
    c[degree - 1] = 0;
    rX[degree - 1] = (8.0 * knots.get(degree - 1).getX()) + knots.get(degree).getX();
    rY[degree - 1] = (8.0 * knots.get(degree - 1).getY()) + knots.get(degree).getY();

    /* solves Ax=b with the Thomas algorithm */
    for (int i = 1; i < degree; i++) {
      double m = a[i] / b[i - 1]; // temporary variable
      b[i] -= m * c[i - 1];
      rX[i] -= m * rX[i - 1];
      rY[i] -= m * rY[i - 1];
    }

    points1[degree - 1] = new Pose(rX[degree - 1] / b[degree - 1],
        rY[degree - 1] / b[degree - 1]);
    for (int i = degree - 2; i >= 0; --i) {
      points1[i] = new Pose((rX[i] - (c[i] * points1[i + 1].getX())) / b[i],
          (rY[i] - (c[i] * points1[i + 1].getY())) / b[i]);
    }

    /* we have p1, now compute p2 */
    for (int i = 0; i < (degree - 1); i++) {
      points2[i] = new Pose((2.0 * knots.get(i + 1).getX()) - points1[i + 1].getX(),
          (2.0 * knots.get(i + 1).getY()) - points1[i + 1].getY());
    }

    points2[degree - 1] = new Pose(
        0.5 * (knots.get(degree).getX() + points1[degree - 1].getX()),
        0.5 * (knots.get(degree).getY() + points1[degree - 1].getY()));

    List<List<Pose>> controlPoints = new ArrayList<>(degree);

    if(startAngle != null) {
      points1[0] = points1[0].rotate(knots.get(0), startAngle, isBackwards(),
          startScale);
    }
    if(endAngle != null) {
      points2[degree - 1] = points2[degree - 1]
          .rotate(knots.get(knots.size() - 1), endAngle, !isBackwards(),
              endScale);
    }

    for (int i = 0; i < degree; i++) {
      List<Pose> segmentControlPoints = new ArrayList<>(getPathNumberOfSteps());
      Collections.addAll(segmentControlPoints, knots.get(i), points1[i], points2[i],
          knots.get(i + 1));
      Collections.addAll(controlPoints, segmentControlPoints);
    }
    return controlPoints;
  }

  private List<SwerveBezierCurve> createBezierCurves(SwervePathData startPathData,
      List<List<Pose>> pathControlPoints) {
    double nextStartVelocity;
    double nextEndVelocity;
    SwerveState nextStartSwerveState;
    SwerveState nextEndSwerveState;
    SwervePathData nextStartPathData = startPathData;
    ListIterator<List<Pose>> pointIterator = pathControlPoints.listIterator();

    List<Double> angleList = new LinkedList<>();

    angleList.add(startSwerveState.getCenterAngle());

    if(!useKnotAngles) {
      double stepAngle =
          (endSwerveState.getCenterAngle() - startSwerveState.getCenterAngle()) / pathControlPoints
              .size();
      for (int i = 0; i < pathControlPoints.size(); i++) {
        angleList.add(angleList.get(i) + stepAngle);
      }
    } else {
      for(int i = 0; i < pathControlPoints.size() - 1; i++) {
        angleList.add(pathControlPoints.get(i).get(3).getAngle());
      }
      angleList.add(endSwerveState.getCenterAngle());
    }

    List<SwerveBezierCurve> bezierCurves = new LinkedList<>();

    while (pointIterator.hasNext()) {
      nextStartVelocity = (pointIterator.nextIndex() == 0) ? startSwerveState.getTranslationalVelocity() : getVCruise();
      nextEndVelocity =
          (pointIterator.nextIndex() == (pathControlPoints.size() - 1)) ? endSwerveState.getTranslationalVelocity()
              : getVCruise();
      nextStartSwerveState = new SwerveState(nextStartVelocity,
          angleList.get(pointIterator.nextIndex()));
      nextEndSwerveState = new SwerveState(nextEndVelocity,
          angleList.get(pointIterator.nextIndex() + 1));

      SwerveBezierCurve curve = new SwerveBezierCurve(kinematicParameters, nextStartSwerveState,
          nextEndSwerveState, nextStartPathData, pointIterator.next());

//			System.out.println(curve.getKeyPoints().size());
      bezierCurves.add(curve);
      nextStartPathData = curve.getSwervePathData().getLast();
    }

    return bezierCurves;
  }

  /**
   * Stitches the bezier curve path data to make the single spline
   *
   */
  private void stitchPathData() {
    for(SwerveBezierCurve swerveBezierCurve : definingBezierCurves) {
      swervePathData.addAll(swerveBezierCurve.getSwervePathData());
    }
  }
}
