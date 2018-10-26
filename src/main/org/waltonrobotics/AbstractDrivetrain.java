package org.waltonrobotics;

import edu.wpi.first.wpilibj.command.Subsystem;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Properties;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.controller.MotionController;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.RobotPair;
import org.waltonrobotics.controller.State;
import org.waltonrobotics.motion.Path;

/**
 * Extend this in your drivetrain, and use the methods inside to set up spline motions
 *
 * @author Russell Newton, Marius Juston, Walton Robotics
 */
public abstract class AbstractDrivetrain extends Subsystem {
//TODO use the java.util.Properties class to save and load the drivetrain constants to a file


  private static final File CONFIG_FILE = new File("config.properties");
  private static final Properties DRIVETRAIN_PROPERTIES = new Properties();
  private final MotionController controller;
  private final MotionLogger motionLogger;
  private Pose actualPosition = new Pose(0, 0, 0);
  private RobotPair previousLengths;
  private double actualPositionTime;
  private PathData currentState;
  private PathData previousState;



  /**
   * Create the static drivetrain after creating the motion logger so you can use the MotionContoller
   */
  protected AbstractDrivetrain(MotionLogger motionLogger) {
    this.motionLogger = motionLogger;
    controller = new MotionController(this);
    SimpleMotion.setDrivetrain(this);
    previousLengths = getWheelPositions();

    previousState = new PathData(
        new State(previousLengths.getLeft(), 0, 0),
        new State(previousLengths.getRight(), 0, 0),
        actualPosition, previousLengths.getTime());

    setEncoderDistancePerPulse();
  }

  /**
   * Drivetrain properties are saved here
   */
  public static Properties getDrivetrainProperties() {
    return DRIVETRAIN_PROPERTIES;
  }

  public MotionLogger getMotionLogger() {
    return motionLogger;
  }

  /**
   * return a new robot pair with left.getDistance(), right.getDistance()
   *
   * @return a RobotPair with the encoder distances;
   */
  public abstract RobotPair getWheelPositions();

  /**
   * return the width of t he robot from teh outside of the wheel on the left side and the right side
   *
   * @return a double informing the width of the robot
   */
  public abstract double getRobotWidth();

  /**
   * Reset the encoders here
   */
  public abstract void reset();

  /**
   * @return whether or not the MotionController is running
   */
  public final boolean getControllerStatus() {
    return controller.isRunning();
  }

  /**
   * Starts the MotionController
   */
  public final void startControllerMotion(Pose startPosition) {
    controller.setStartPosition(startPosition);
    controller.enableScheduler();
  }

  /**
   * Cancels the MotionController
   */
  public final void cancelControllerMotion() {
    controller.stopScheduler();
  }

  /**
   * Clears the current queue
   */
  public final void clearControllerMotions() {
    controller.clearMotions();
  }

  /**
   * @param paths - paths to add to the MotionController queue
   */
  public final void addControllerMotions(Path... paths) {
    controller.addPaths(paths);
  }

  /**
   * @return if the robot has completed all motions
   */
  public final boolean isControllerFinished() {
    return controller.isFinished();
  }

  /**
   * Set the motor speeds here
   */
  public abstract void setSpeeds(double leftPower, double rightPower);

  /**
   * Set the encoder distances per pulse here
   */
  public abstract void setEncoderDistancePerPulse();

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
  public abstract double getILag();

  public abstract double getIAng();

  public double getPercentPathDone(Path path) {
    return controller.getPercentDone(path);
  }

  /**
   * This returns the max velocity the robot can achieve.
   * <br>
   * This value can also be found using the <a href=https://github.com/NamelessSuperCoder/Motion-Profiller-Log-Display>Motion
   * Log Viewer</a> using a motion that is long and straight and has a high max velocity.
   *
   * @return the max acceleration the robot can achieve.
   */
  public abstract double getMaxVelocity();

  /**
   * This returns the max acceleration the robot can be achieve
   *
   * @return the max acceleration the robot can achieve
   */
  public abstract double getMaxAcceleration();

  /**
   * Returns the approximate actual location of the robot from the origin (0,0)
   *
   * @return the approximate location of the robot from (0,0)
   */
  public Pose getActualPosition() {
    return actualPosition;
  }

  /**
   * Sets the start location of the robot instead of the origin of (0,0)
   */
  public void setStartingPosition(Pose startingPosition) {
    controller.setStartPosition(startingPosition);
  }

  @Override
  public String toString() {
    return "AbstractDrivetrain{" +
        "controller=" + controller +
        ", motionLogger=" + motionLogger +
        ", actualPosition=" + actualPosition +
        ", previousLengths=" + previousLengths +
        ", actualPositionTime=" + actualPositionTime +
        ", currentState=" + currentState +
        ", previousState=" + previousState +
        '}';
  }

  /**
   * Gets the actual position time on the robot
   */
  public double getActualPositionTime() {
    return actualPositionTime;
  }

  /**
   * Gets the current state of the robot (wheel encoder distances, velocities, accelerations and jerks), its position
   * (x, y, angle) and the time it was calculated
   */
  public PathData getCurrentRobotState() {
    return currentState;
  }

  @Override
  public void periodic() {
//		Gets the current predicted actual position
    RobotPair wheelPosition = getWheelPositions();
    actualPosition = controller
        .updateActualPosition(wheelPosition, previousLengths, actualPosition);
    actualPositionTime = wheelPosition.getTime();

//		Found change in time between the different periodic calls
    double deltaTime = wheelPosition.getTime() - previousState.getTime();

////		Velocity is the change of distance over time
//		double lVelocity = (wheelPosition.getLeft() - previousState.getLeftState().getLength()) / deltaTime;
//		double rVelocity = (wheelPosition.getRight() - previousState.getRightState().getLength()) / deltaTime;
//
////		Acceleration is the change of velocity over time
//		double lAcceleration = (lVelocity - previousState.getLeftState().getVelocity()) / deltaTime;
//		double rAcceleration = (rVelocity - previousState.getRightState().getVelocity()) / deltaTime;
//
////		Jerk is the change of acceleration over time
//		double lJerk = (lAcceleration - previousState.getLeftState().getAcceleration()) / deltaTime;
//		double rJerk = (rAcceleration - previousState.getRightState().getAcceleration()) / deltaTime;

    currentState = new PathData(
        State.calculateConstants(previousState.getLeftState(), wheelPosition.getLeft(), deltaTime),
        State
            .calculateConstants(previousState.getRightState(), wheelPosition.getRight(), deltaTime),
//			new State(wheelPosition.getLeft(), lVelocity, lAcceleration, lJerk),
//			new State(wheelPosition.getRight(), rVelocity, rAcceleration, rJerk),
        actualPosition,
        actualPositionTime);

    previousState = currentState;
    previousLengths = wheelPosition;
  }

  //	TODO figure out how to load the properties (might need to restructure AbstractDrivetrain)

  /**
   * Saves the drivetrain properties to a config.properties file
   */
  public final void saveProperties() {
    DRIVETRAIN_PROPERTIES.setProperty("robotWidth", String.valueOf(getRobotWidth()));
    DRIVETRAIN_PROPERTIES.setProperty("kV", String.valueOf(getKV()));
    DRIVETRAIN_PROPERTIES.setProperty("kAcc", String.valueOf(getKAcc()));
    DRIVETRAIN_PROPERTIES.setProperty("kK", String.valueOf(getKK()));
    DRIVETRAIN_PROPERTIES.setProperty("kS", String.valueOf(getKS()));
    DRIVETRAIN_PROPERTIES.setProperty("kAng", String.valueOf(getKAng()));
    DRIVETRAIN_PROPERTIES.setProperty("kL", String.valueOf(getKL()));
    DRIVETRAIN_PROPERTIES.setProperty("iLag", String.valueOf(getILag()));
    DRIVETRAIN_PROPERTIES.setProperty("iAng", String.valueOf(getIAng()));
    DRIVETRAIN_PROPERTIES.setProperty("maxVelocity", String.valueOf(getMaxVelocity()));
    DRIVETRAIN_PROPERTIES.setProperty("maxAcceleration", String.valueOf(getMaxAcceleration()));

    try
        (FileWriter writer = new FileWriter(CONFIG_FILE)) {
      DRIVETRAIN_PROPERTIES.store(writer, "drive train settings");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
