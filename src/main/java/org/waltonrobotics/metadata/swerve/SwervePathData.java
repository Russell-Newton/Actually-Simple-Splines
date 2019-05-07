package org.waltonrobotics.metadata.swerve;

import org.waltonrobotics.metadata.Pose;

/**
 * @author Russell Newton
 **/
public class SwervePathData {

  private final SwerveState swerveState;
  private final Pose centerPose;
  private final double time;

  public SwervePathData(SwerveState swerveState, Pose centerPose, double time) {
    this.swerveState = swerveState;
    this.centerPose = centerPose;
    this.time = time;
  }

  public SwervePathData() {
    this(new SwerveState(), new Pose(), 0);
  }

  public SwerveState getSwerveState() {
    return swerveState;
  }

  public Pose getCenterPose() {
    return centerPose;
  }

  public double getTime() {
    return time;
  }

  @Override
  public String toString() {
    return "SwervePathData{" +
        "swerveState=" + swerveState.toString() +
        ", centerPose=" + centerPose.toString() +
        ", time=" + time +
        "}";
  }
}
