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
	private int period;
	private Path currentPath = null;
	private State[] staticState;
	private double startTime;
	private final AbstractDrivetrain drivetrain;
	private final double kV;
	private final double kK;
	private final double kA;
	private final double kP;

	/**
	 * @param drivetrain
	 *            - the drivetrain to use the AbstractDrivetrain methods from
	 */
	public MotionController(AbstractDrivetrain drivetrain) {
		running = false;

		controller = new Timer();
		this.period = 5;
		controller.schedule(new MotionTask(), 0L, (long) period);

		this.drivetrain = drivetrain;
		this.kV = drivetrain.getKV();
		this.kK = drivetrain.getKK();
		this.kA = drivetrain.getKA();	
		this.kP = drivetrain.getKP();

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
				currentState = currentPath.interpolatePosition(time);
			} else {
				currentState = staticState;
			}
			
			if(currentPath.isFinished) {
				currentPath = paths.pollFirst();
				if(currentPath != null) {
					currentPath.setStartingWheelPositions(drivetrain.getWheelPositions());
				}	else	{
					System.out.println("Done with motions! :)");
				}
				startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
				staticState = new State[] { new State(drivetrain.getWheelPositions().getLeft(), 0, 0),
						new State(drivetrain.getWheelPositions().getRight(), 0, 0) };
				currentState = staticState;
				return;
			}

			synchronized (this) {
				// feed forward
				leftPower += (kV * currentState[0].getVelocity() + kK)
						+ kA * currentState[0].getAcceleration();
				rightPower += (kV * currentState[1].getVelocity() + kK)
						+ kA * currentState[1].getAcceleration();
				// feed back
				leftPower += kP * (currentState[0].getLength() - wheelPositions.getLeft());
				rightPower += kP * (currentState[1].getLength() - wheelPositions.getRight());

			}

			leftPower = Math.max(-1, Math.min(1, leftPower));
			rightPower = Math.max(-1, Math.min(1, rightPower));

			drivetrain.setSpeeds(leftPower, rightPower);
			System.out.println("Current left: " + wheelPositions.getLeft() + " Current Right: " + wheelPositions.getRight());
		}
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
			startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
			staticState = new State[] { new State(drivetrain.getWheelPositions().getLeft(), 0, 0),
					new State(drivetrain.getWheelPositions().getRight(), 0, 0) };
			Path newPath = paths.poll();
			if (newPath != null) {
				currentPath = newPath;
				running = true;
			}
			currentPath.setStartingWheelPositions(drivetrain.getWheelPositions());
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
