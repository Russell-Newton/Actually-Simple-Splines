package org.waltonrobotics.motion.swerve;

import java.util.LinkedList;
import java.util.List;
import org.waltonrobotics.metadata.KinematicParameters;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.swerve.SwervePathData;
import org.waltonrobotics.metadata.swerve.SwerveState;
import org.waltonrobotics.motion.Path;

/**
 * @author Russell Newton
 **/
public abstract class SwervePath extends Path {

  protected KinematicParameters kinematicParameters;
  protected SwerveState startSwerveState;
  protected SwerveState endSwerveState;
  protected final LinkedList<SwervePathData> swervePathData;


  protected SwervePath(KinematicParameters kinematicParameters, SwerveState startSwerveState,
      SwerveState endSwerveState, List<Pose> keyPoints) {
    super(1, 1, false, keyPoints);
    this.kinematicParameters = kinematicParameters;
    this.startSwerveState = startSwerveState;
    this.endSwerveState = endSwerveState;
    swervePathData = new LinkedList<>();
  }

  public KinematicParameters getKinematicParameters() {
    return kinematicParameters;
  }

  public LinkedList<SwervePathData> getSwervePathData() {
    return swervePathData;
  }

  @Override
  protected List<double[]> defineBezierCoefficients() {
    List<double[]> coefficients = super.defineBezierCoefficients();
    
    double[] angleCoefficients = new double[]{startSwerveState.getCenterAngle(),
        endSwerveState.getCenterAngle() - startSwerveState.getCenterAngle()};

    coefficients.add(angleCoefficients);

    return coefficients;
  }
}
