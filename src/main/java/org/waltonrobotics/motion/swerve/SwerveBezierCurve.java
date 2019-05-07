package org.waltonrobotics.motion.swerve;

import static java.lang.Math.PI;
import static org.waltonrobotics.util.Polynomial.getPoint;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.waltonrobotics.metadata.KinematicParameters;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.swerve.SwervePair;
import org.waltonrobotics.metadata.swerve.SwervePathData;
import org.waltonrobotics.metadata.swerve.SwerveState;
import org.waltonrobotics.util.Vector;

/**
 * @author Russell Newton
 **/
public class SwerveBezierCurve extends SwervePath{

  private List<double[]> coefficients;
  private final SwervePathData startPathData;
  private double curveLength;

  public SwerveBezierCurve(KinematicParameters translationParameters, SwerveState startSwerveState, SwerveState endSwerveState,
      SwervePathData startPathData, List<Pose> controlPoints) {
    super(translationParameters, startSwerveState, endSwerveState,
        controlPoints);
    this.startPathData = startPathData;
    coefficients = defineBezierCoefficients();

    createPath();
  }

  public SwerveBezierCurve(KinematicParameters translationParameters, SwerveState startSwerveState,
      org.waltonrobotics.metadata.swerve.SwerveState endSwerveState, List<Pose> controlPoints) {
    this(translationParameters, startSwerveState, endSwerveState,
        new SwervePathData(), controlPoints);
  }

  private void createPath() {
    pathPoints = createPoints();
    curveLength = getPathLength();
    setData(startPathData);
  }

  private List<Pose> createPoints() {
    List<Pose> pathPoints = new LinkedList<>();
//    System.out.println(Arrays.toString(coefficients.get(2)));

    for (double i = 0; i <= getPathNumberOfSteps(); i++) {

      pathPoints.add(getPoint(coefficients.get(0), coefficients.get(1), coefficients.get(2),
          i / getPathNumberOfSteps()));
    }

    return pathPoints;
  }

  private void setData(SwervePathData startData) {
    double passedLength = 0;
    for(int i = 1; i < pathPoints.size(); i++) {
      Pose nextPosition = pathPoints.get(i);
      passedLength += nextPosition.distance(startData.getCenterPose());
      startData = calculateData(startData, nextPosition, passedLength);
      swervePathData.add(startData);
    }
  }

  private SwervePathData calculateData(SwervePathData previousPathData, Pose nextPosition,
      double passedLength) {
    Pose previousCenter = previousPathData.getCenterPose();
    SwerveState previousSwerveState = previousPathData.getSwerveState();
    double previousTime = previousPathData.getTime();
    double previousAngle = previousPathData.getSwerveState().getCenterAngle();
    double nextAngle = nextPosition.getAngle();

    Vector dTranslation = Vector.displacementVector(previousCenter, nextPosition);
    double dTime = dTranslation.getMagnitude() / kinematicParameters.getMaxVelocity();
    double velocity = dTranslation.getMagnitude() / dTime;

    double vAccelerating;
    double vDecelerating;

    vAccelerating = Math
        .sqrt(StrictMath.pow(startSwerveState.getTranslationalVelocity(), 2.0) +
            (getKinematicParameters().getMaxAcceleration() * passedLength));
    vDecelerating = Math
        .sqrt(
            StrictMath.pow(endSwerveState.getTranslationalVelocity(), 2.0) +
                (getKinematicParameters().getMaxAcceleration() * (curveLength - passedLength)));
//    System.out.println(vAccelerating + " " + vDecelerating);

    if ((vAccelerating < velocity) && (vAccelerating < vDecelerating)) {
      dTime = dTranslation.getMagnitude() / vAccelerating;
    }
    if ((vDecelerating < velocity) && (vDecelerating < vAccelerating)) {
      dTime = dTranslation.getMagnitude() / vDecelerating;
    }

      Vector centerVelocity = dTranslation.scale(1.0 / dTime);

      double dAngle = nextAngle - previousAngle;
      double omega = dAngle / dTime;
      Vector radius = Vector.vectorFromXandY(getRobotWidth() / 2, getRobotLength() / 2);
      Vector rotationalVelocity = radius.scale(omega).rotate(-PI / 2);

      List<SwervePair> swervePairs = new LinkedList<>();
      for(int i = 0; i < 4; i++) {
        Vector resultantVelocity = centerVelocity.findResultant(rotationalVelocity);
        swervePairs.add(SwervePair.swervePairFromVector(resultantVelocity));
        rotationalVelocity = rotationalVelocity.rotate(-PI / 2);
      }

    SwerveState swerveState = new SwerveState(centerVelocity.getMagnitude(), nextAngle,
        swervePairs);
    return new SwervePathData(swerveState, nextPosition, previousTime + dTime);

  }

}
