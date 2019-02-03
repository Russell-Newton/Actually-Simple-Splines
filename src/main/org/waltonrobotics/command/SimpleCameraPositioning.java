package org.waltonrobotics.command;

import org.waltonrobotics.controller.CameraData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.Spline;

public class SimpleCameraPositioning extends SimpleMotion {


  public SimpleCameraPositioning(Pose robotCameraEstimatedPosition, Pose targetPosition, double maxVelocity,
      double maxAcceleration,
      double startVelocity, double endVelocity, boolean isBackwards) {
    super(new Spline(
        maxVelocity,
        maxAcceleration,
        startVelocity,
        endVelocity,
        isBackwards,
        robotCameraEstimatedPosition,
        targetPosition
    ));
  }

  public SimpleCameraPositioning(CameraData cameraData, double maxVelocity, double maxAcceleration,
      double startVelocity, double endVelocity, boolean isBackwards) {
    this(cameraData.getCameraPose(), maxVelocity, maxAcceleration, startVelocity, endVelocity, isBackwards);
  }

  public SimpleCameraPositioning(Pose cameraData, double maxVelocity, double maxAcceleration,
      double startVelocity, double endVelocity, boolean isBackwards) {
    this(cameraData, Pose.ZERO.offset(-getDrivetrain().getRobotLength() / 2.0), maxVelocity, maxAcceleration, startVelocity,
        endVelocity, isBackwards);
  }

  public static SimpleCameraPositioning toCameraTarget(CameraData cameraData) {
    return new SimpleCameraPositioning(cameraData, getDrivetrain().getMaxVelocity(),
        getDrivetrain().getMaxAcceleration(), 0, 0, false);
  }

  public static SimpleCameraPositioning toCameraTarget(Pose estimatedRobotPosition) {
    return new SimpleCameraPositioning(estimatedRobotPosition, getDrivetrain().getMaxVelocity(),
        getDrivetrain().getMaxAcceleration(), 0, 0, false);
  }
}