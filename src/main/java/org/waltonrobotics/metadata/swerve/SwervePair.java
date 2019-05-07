package org.waltonrobotics.metadata.swerve;

import org.waltonrobotics.util.Vector;

/**
 * @author Russell Newton
 **/
public class SwervePair {

  private final double velocity;
  private final double angle;

  public SwervePair(double velocity, double angle) {
    this.velocity = velocity;
    this.angle = angle;
  }

  public SwervePair() {
    this(0, 0);
  }

  public double getVelocity() {
    return velocity;
  }

  public double getAngle() {
    return angle;
  }

  @Override
  public String toString() {
    return "Swerve Pair{" +
        "velocity=" + velocity +
        ", angle=" + angle +
        "}";
  }

  public static SwervePair swervePairFromVector(Vector vector) {
    return new SwervePair(vector.getMagnitude(), vector.getAngle());
  }
}
