package org.waltonrobotics;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.controller.MotionController;
import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.RobotPair;

/**
 * Extend this in your drivetrain, and use the methods inside to set up spline motions
 *
 * @author Russell Newton, Walton Robotics
 */
public abstract class AbstractDrivetrain extends Subsystem {

	private final MotionController controller;
	private final MotionLogger motionLogger;

	/**
	 * Create the static drivetrain after creating the motion logger so you can use the MotionContoller
	 */
	protected AbstractDrivetrain(MotionLogger motionLogger) {
		this.motionLogger = motionLogger;
		controller = new MotionController(this);
		SimpleMotion.setDrivetrain(this);
	}

	protected AbstractDrivetrain() {
		this(new MotionLogger(""));
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
		controller.enableScheduler(startPosition);
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
	 * This is the constant for angle control. Using the MotionLogger, KT is correct when the angle and cross track
	 * errors approach 0.
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
	 * @see
	 */
	public abstract double getMaxVelocity();

	/**
	 * This returns the max acceleration the robot can be achieve
	 *
	 * @return the max acceleration the robot can achieve
	 */
	public abstract double getMaxAcceleration();

	@Override
	public String toString() {
		return "AbstractDrivetrain{" +
			"controller=" + controller +
			", motionLogger=" + motionLogger +
			'}';
	}
}
