package org.waltonrobotics.metadata;

/**
 * @author Russell Newton
 **/
public class KinematicParameters {

  private final double maxVelocity;
  private final double maxAcceleration;

  public KinematicParameters(double maxVelocity, double maxAcceleration) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public double getMaxAcceleration() {
    return maxAcceleration;
  }

  @Override
  public String toString() {
    return "KinematicParameters{" +
        "maxVelocity=" + maxVelocity +
        ", maxAcceleration=" + maxAcceleration +
        "}";
  }
}
