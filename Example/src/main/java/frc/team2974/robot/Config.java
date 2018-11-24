package frc.team2974.robot;

import org.waltonrobotics.controller.Pose;

/**
 * This holds many constant values for all parts of the robot. It increases efficiency and effectiveness of the code.
 */
public final class Config {

  private Config() {
  }

  public enum Robot {
    COMPETITION(0.7800, 0.0002045);


    private final double robotWidth;
    private final double distancePerPulse;

    Robot(double robotWidth, double distancePerPulse) {

      this.robotWidth = robotWidth;
      this.distancePerPulse = distancePerPulse;
    }

    public double getRobotWidth() {
      return robotWidth;
    }

    public double getDistancePerPulse() {
      return distancePerPulse;
    }
  }

  public static final class Hardware {

    public static final int LEFT_MOTOR_CHANNEL = 0; // pwm
    public static final int LEFT_ENCODER_CHANNEL1 = 2; // for first digital input
    public static final int LEFT_ENCODER_CHANNEL2 = 3; // for second digital input

    public static final int RIGHT_MOTOR_CHANNEL = 1; // pwm
    public static final int RIGHT_ENCODER_CHANNEL1 = 0; // digital
    public static final int RIGHT_ENCODER_CHANNEL2 = 1; // digital

    private Hardware() {
    }
  }

  public static final class Input {

    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;
    public static final int GAMEPAD_PORT = 2;

    public static final double THROTTLE_THRESHOLD = .3;

    private Input() {
    }
  }

  public static final class Path {


    public static final double VELOCITY_MAX = 4; /* 3.075548163 TESTED MAX VELOCITY*/ //3 m/s
    public static final double ACCELERATION_MAX = 3.5; //2 m/s^2

    private Path() {
    }

    /**
     * Used for R -> L points. Easy and fast.
     *
     * @param p the point to negate its x
     * @return a new point with the x negated from p
     */
    private static Pose negateX(Pose p) {
      double angle = p.getAngle();
      // the new angle is the original angle but x is negated
      double newAngle = StrictMath.atan2(StrictMath.sin(angle), -StrictMath.cos(angle));
      if (newAngle < 0) {
        newAngle += 2 * Math.PI;
      }
      return new Pose(-p.getX(), p.getY(), newAngle);
    }
  }

  public static final class MotionConstants {

    public static final double KV = 0.194350; // 0.267
    public static final double KAcc = 0.067555; //0
    public static final double KK = 0.193466; // 0

    public static final double KS = 2;//1
    public static final double KAng = 1;//1
    public static final double KL = 2; //2

    public static final double IL = 0.00; // 0.01
    public static final double IAng = 0.005; // 0.01

    private MotionConstants() {
    }
  }
}
