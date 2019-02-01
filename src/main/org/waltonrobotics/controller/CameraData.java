package org.waltonrobotics.controller;

public class CameraData {
  private final Pose cameraPose;
  private final double height;
  private final int numberOfTargets;
  private final double t;

  public CameraData(double x, double y, double height, double angle, int numberOfTargets, double t) {
    cameraPose = new Pose(x, y, angle);
    this.height = height;
    this.numberOfTargets = numberOfTargets;
    this.t = t;
  }

  public CameraData(int numberOfTargets) {
    this(0, 0, 0, 0, numberOfTargets, -1);
  }

  public CameraData() {
    this(0, 0, 0, 0, 0, -1);
  }

  public Pose getCameraPose() {
    return cameraPose;
  }

  public double getHeight() {
    return height;
  }

  public int getNumberOfTargets() {
    return numberOfTargets;
  }

  public double getT() {
    return t;
  }
}
