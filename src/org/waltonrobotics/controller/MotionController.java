package org.waltonrobotics.controller;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

import org.waltonrobotics.AbstractDrivetrain;

/**
 * Sends power to the wheels
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class MotionController {

	private class MotionTask extends TimerTask {

		@Override
		public void run() {
			if (currentPath != null) {
				calculateSpeeds();
			}
		}

	}

	private BlockingDeque<Path> paths = new LinkedBlockingDeque<Path>();
	private Timer controller;
	private boolean running;
	private int currentStep = 0;
	private int period;
	private Path currentPath = null;
	private State[] staticState;
	private double startTime;
	private final AbstractDrivetrain drivetrain;
	private final double KVelocity;
	private final double KScaling;
	private final double KAcceleration;
	private final double KPower;

	/**
	 * @param drivetrain - the drivetrain to use the AbstractDrivetrain methods from
	 */
	public MotionController(AbstractDrivetrain drivetrain) {
		running = false;
		
		controller = new Timer();
		this.period = 5;
		controller.schedule(new MotionTask(), 0L, (long) period);
		
		this.drivetrain = drivetrain;
		this.KVelocity = 0.5;
		this.KScaling = 0;
		this.KAcceleration = 0.1;
		this.KPower = 20;

	}

	/**
	 * Adds a path to the path queue
	 * 
	 * @param paths
	 *            - the paths to add to the queue
	 */
	public void addPaths(Path... paths) {
		for (Path path : paths) {
			this.paths.addLast(path);
		}
	}

	/**
	 * Calculates the powers to send to the wheels
	 */
	private void calculateSpeeds() {

		double leftPower = 0;
		double rightPower = 0;
		boolean enabled;

		synchronized (this) {
			enabled = this.running;
		}

		if (enabled) {

			double time = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime;

			RobotPair wheelPositions = drivetrain.getWheelPositions();

			State[] currentState;
			if (currentPath != null) {
				currentState = interpolatePosition(time);
			} else {
				currentState = staticState;
			}
			try {
				if (currentState == null) {
					currentStep++;
					currentState = interpolatePosition(time);
				}
			} catch (ArrayIndexOutOfBoundsException e) {
				currentPath = paths.pollFirst();
				drivetrain.reset();
				currentStep = 0;
				startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
				staticState = new State[] { new State(drivetrain.getWheelPositions().getLeft(), 0, 0),
						new State(drivetrain.getWheelPositions().getRight(), 0, 0) };
				return;
			}

			synchronized (this) {
				// feed forward
				leftPower += (KVelocity * currentState[0].getVelocity() + KScaling) + KAcceleration * currentState[0].getAcceleration();
				rightPower += (KVelocity * currentState[1].getVelocity() + KScaling) + KAcceleration * currentState[1].getAcceleration();
				// feed back
				leftPower += KPower * (currentState[0].getLength() - wheelPositions.getLeft());
				rightPower += KPower * (currentState[1].getLength() - wheelPositions.getRight());

			}

			leftPower = Math.max(-1, Math.min(1, leftPower));
			rightPower = Math.max(-1, Math.min(1, rightPower));

			drivetrain.setSpeeds(leftPower, rightPower);
		}
	}

	/**
	 * Calculates parameters for wheel powers based off of location between steps
	 * 
	 * @param time
	 *            - current time since start
	 * @param step
	 * @return a set of left and right lengths, velocities, and accelerations.
	 */
	private State[] interpolatePosition(double time) {
		Point previousLeft = currentPath.getLeftPath()[currentStep];
		Point previousRight = currentPath.getRightPath()[currentStep];
		Point nextLeft = currentPath.getLeftPath()[currentStep + 1];
		Point nextRight = currentPath.getRightPath()[currentStep + 1];

		// Difference in time
		double dTime = nextLeft.getTime() - previousLeft.getTime();
		// Ratio from position to next point
		double ratioPosToNext = (nextLeft.getTime() - time) / dTime;
		// Ratio from previous point to position
		double rationPrevToPos = (time - previousLeft.getTime()) / dTime;
		if (ratioPosToNext == 0) {
			return null;
		}

		// New values are the average of the previous and next point values
		// (Average = sum(value * ratio))
		double lLeft = previousLeft.getLength() * ratioPosToNext + nextLeft.getLength() * rationPrevToPos;
		double vLeft = previousLeft.getVelocity() * ratioPosToNext + nextLeft.getVelocity() * rationPrevToPos;
		double aLeft = previousLeft.getAcceleration() * ratioPosToNext + nextLeft.getAcceleration() * rationPrevToPos;

		double lRight = previousRight.getLength() * ratioPosToNext + nextRight.getLength() * rationPrevToPos;
		double vRight = previousRight.getVelocity() * ratioPosToNext + nextRight.getVelocity() * rationPrevToPos;
		double aRight = previousRight.getAcceleration() * ratioPosToNext
				+ nextRight.getAcceleration() * rationPrevToPos;

		return new State[] { new State(lLeft, vLeft, aLeft), new State(lRight, vRight, aRight) };
	}

	/**
	 * Removes all queued motions
	 */
	public void clearMotions() {
		paths.clear();
	}

	/**
	 * Starts the queue of motions
	 */
	public void enableScheduler() {
		if (!running) {
			currentStep = 0;
			startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
			drivetrain.reset();
			staticState = new State[] { new State(drivetrain.getWheelPositions().getLeft(), 0, 0),
					new State(drivetrain.getWheelPositions().getRight(), 0, 0) };
			Path newPath = paths.poll();
			if (newPath != null) {
				currentPath = newPath;
				System.out.println(currentPath);
				running = true;
			}
		}
	}

	/**
	 * 
	 * @return Whether or not the queue has ended
	 */
	public boolean isFinished() {
		return currentPath == null;
	}

	/**
	 * 
	 * @return Whether or not a motion is running
	 */
	public boolean isRunning() {
		return running;
	}

	/**
	 * Pauses the motions,
	 */
	public void stopScheduler() {
		running = false;
		currentPath = null;
		controller.cancel();
		drivetrain.setSpeeds(0, 0);
	}
}
