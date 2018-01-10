package org.waltonrobotics;

import org.waltonrobotics.controller.MotionController;
import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.RobotPair;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Extend this in your drivetrain, and use the methods inside to set up spline
 * motions
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public abstract class AbstractDrivetrain extends Subsystem {

	static MotionController controller;

	/**
	 * Put your Robot.drivetrain into super()
	 * 
	 * @param drivetrain
	 */
	public AbstractDrivetrain(AbstractDrivetrain drivetrain) {
		controller = new MotionController(drivetrain);
	}

	/**
	 * return a new robot pair with left.getDistance(), right.getDistance()
	 * 
	 * @return a RobotPair with the encoder distances;
	 */
	public abstract RobotPair getWheelPositions();

	/**
	 * Reset the encoders here
	 */
	public abstract void reset();

	/**
	 * @return whether or not the MotionController is running
	 */
	public boolean getControllerStatus() {
		return controller.isRunning();
	}

	/**
	 * Starts the MotionController
	 */
	public void startControllerMotion() {
		controller.enableScheduler();
	}

	/**
	 * Cancels the MotionController
	 */
	public void cancelControllerMotion() {
		controller.stopScheduler();
	}

	/**
	 * @param paths
	 *            - paths to add to the MotionController queue
	 */
	public void addControllerMotions(Path... paths) {
		controller.addPaths(paths);
	}

	/**
	 * @return if the robot has completed all motions
	 */
	public boolean isControllerFinished() {
		return controller.isFinished();
	}

	/**
	 * Set the motor speeds here
	 * 
	 * @param leftPower
	 * @param rightPower
	 */
	public abstract void setSpeeds(double leftPower, double rightPower);

	/**
	 * Set the encoder distances per pulse here
	 */
	public abstract void setEncoderDistancePerPulse();

}
