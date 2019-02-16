package org.waltonrobotics.util;

public abstract class RobotConfig {

  private final String robotName;

  public RobotConfig(String robotName) {
    this.robotName = robotName;
  }

  public String getRobotName() {
    return robotName;
  }

  public abstract EncoderConfig getRightEncoderConfig();

  public abstract EncoderConfig getLeftEncoderConfig();

  public abstract TalonConfig getLeftTalonConfig();

  public abstract TalonConfig getRightTalonConfig();

  public abstract Controls getRightJoystickConfig();

  public abstract Controls getLeftJoystickConfig();

  /**
   * This returns the max velocity the robot can achieve.
   * <br>
   * This value can also be found using the <a href=https://github.com/NamelessSuperCoder/Motion-Profiller-Log-Display>Motion
   * Log Viewer</a> using a motion that is long and straight and has a high max velocity. or it can be calculated by
   * using the (1 - kK)/ kV equation
   *
   * @return the max velocity the robot can achieve.
   */
  public double getMaxVelocity() {
    return (1.0 - getKK()) / getKV();
  }

  /**
   * This returns the max acceleration the robot can be achieve
   *
   * @return the max acceleration the robot can achieve
   */
  public abstract double getMaxAcceleration();

  /**
   * The velocity constant. This is the feed forward multiplier. Using the MotionLogger, KV is correct if lag error
   * levels out.
   *
   * @return KV
   */
  public abstract double getKV();

  /**
   * The acceleration constant. This adds to the feed forward by giving a slight boost while accelerating or
   * decelerating. Make this a very small number greater than 0 if anything.
   *
   * @return KAcc
   */
  public abstract double getKAcc();

  /**
   * This constant gives a slight boost to the motors. Make this a very small number greater than 0 if anything.
   *
   * @return KK
   */
  public abstract double getKK();

  /**
   * This is the constant for steering control. Using the MotionLogger, KS is correct when the cross track error
   * provides a steady oscillation. Set this before KAng.
   *
   * @return KS
   */
  public abstract double getKS();

  /**
   * This is the constant for angle control. Using the MotionLogger, KT is correct when the angle and cross track errors
   * approach 0.
   *
   * @return KAng
   */
  public abstract double getKAng();

  /**
   * This is the lag constant. Using the MotionLogger, KL is correct when the lag error is (close to) 0.
   *
   * @return KL
   */
  public abstract double getKL();

  //TODO do documentation for theses variables
  public double getILag() {
    return 0;
  }

  public double getIAng() {
    return 0;
  }

  /**
   * return the width of t he robot from teh outside of the wheel on the left side and the right side
   *
   * @return a double informing the width of the robot
   */
  public abstract double getRobotWidth();

  /**
   * return the length of the robot from the outside of the bumpers
   *
   * @return a double informing the width of the robot
   */
  public abstract double getRobotLength();


  /**
   * return the length of the robot from the outside of the bumpers
   *
   * @return a double informing the width of the robot
   */
  public abstract boolean isCurrentRobot();

}
