package org.waltonrobotics.util;

import org.waltonrobotics.metadata.Pose;

public class Vector {

  private double magnitude;
  private double angle;
  private double xComponent;
  private double yComponent;

  /**
   * Create a vector from a magnitude and angle
   *
   * @param magnitude the magnitude of the vector
   * @param angle the angle (in radians) of the vector
   */
  public Vector(double magnitude, double angle) {
    xComponent = magnitude * Math.cos(angle);
    yComponent = magnitude * Math.sin(angle);

    this.magnitude = Math.sqrt(Math.pow(xComponent, 2) + Math.pow(yComponent, 2));
    this.angle = Math.atan2(yComponent, xComponent);
  }

  /**
   * Create a vector from an x and y component
   *
   * @param x the x component of the vector
   * @param y the y component of the vector
   * @return a Vector {@literal <x, y>}
   */
  public static Vector vectorFromXandY(double x, double y) {
    double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    double angle = Math.atan2(y, x);
    return new Vector(magnitude, angle);
  }

  public static Vector vectorFromPose(Pose pose) {
    return vectorFromXandY(pose.getX(), pose.getY());
  }

  public static Pose poseFromVector(Vector vector) {
    return new Pose(vector.getxComponent(), vector.getyComponent());
  }

  public Pose convertToPose() {
    return new Pose(xComponent, yComponent);
  }

  /**
   * Combines this vector with another vector, capping the magnitude
   *
   * @param v2 the other vector
   * @param maxMagnitude the magnitude cap
   * @return the resultant vector
   */
  public Vector findResultant(Vector v2, double maxMagnitude) {
    double combinedX = this.getxComponent() + v2.getxComponent();
    double combinedY = this.getyComponent() + v2.getyComponent();
    double combinedMagnitude = Math.min(Math.sqrt(Math.pow(combinedX, 2) + Math.pow(combinedY, 2)),
        maxMagnitude);
    double combinedAngle = StrictMath.atan2(combinedY, combinedX);

    return new Vector(combinedMagnitude, combinedAngle);
  }

  /**
   * Combines this vector with another vector without a max magnitude
   */
  public Vector findResultant(Vector v2) {
    return this.findResultant(v2, Double.MAX_VALUE);
  }

  public double getMagnitude() {
    return magnitude;
  }

  public double getAngle() {
    return angle;
  }

  public double getxComponent() {
    return xComponent;
  }

  public double getyComponent() {
    return yComponent;
  }

  /**
   * Scales this Vector
   *
   * @param scalar the scalar
   * @return this Vector, multiplied by scalar
   */
  public Vector scale(double scalar) {
    return new Vector(this.magnitude * scalar, this.angle);
  }

  /**
   * Multiplies xComponent with -1
   */
  public void negateXComponent() {
    xComponent *= -1;
  }

  /**
   * Multiplies yComponent with -1
   */
  public void negateYComponent() {
    yComponent *= -1;
  }

  /**
   * @return the unit Vector of this Vector
   */
  public Vector normalize() {
    return scale(1.0 / magnitude);
  }

  public String toString() {
    return String.format("%.3f at %.3f degrees", this.magnitude, Math.toDegrees(this.angle));
  }

  public Vector rotate(double angle) {
    return new Vector(magnitude, angle + this.angle);
  }

  public static Vector displacementVector(Pose a, Pose b) {
    double dx = b.getX() - a.getX();
    double dy = b.getY() - a.getY();

    return vectorFromXandY(dx, dy);
  }
}
