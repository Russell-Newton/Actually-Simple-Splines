package org.waltonrobotics.controller;

import java.util.Collections;
import java.util.LinkedList;
import java.util.ListIterator;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.MotionLogger;

/**
 * Controls Path motions
 *
 * @author Russell Newton, Walton Robotics
 */
public class MotionController {

	private final AbstractDrivetrain drivetrain;
	private final double kV;
	private final double kK;
	private final double kAcc;
	private final double kS;
	private final double kL;
	private final double kAng;
	private final BlockingDeque<Path> paths = new LinkedBlockingDeque<>();
	private final int period;
	private final MotionLogger motionLogger;
	private Timer controller;
	private boolean running;
	private Path currentPath;
	private PathData staticPathData;
	private Pose actualPosition;
	private PathData targetPathData;
	private RobotPair previousLengths;
	private RobotPair startingWheelPositions;
	private ListIterator<PathData> pdIterator;
	private PathData pdPrevious;
	private PathData pdNext;
	private ErrorVector errorVector;
	private RobotPair powers;
	private TimerTask currentTimerTask;

	/**
	 * @param drivetrain - the drivetrain to use the AbstractDrivetrain methods from
	 * @param robotWidth - the robot width from the outside of the wheels
	 * @param motionLogger - the MotionLogger from the AbstractDrivetrain
	 */
	public MotionController(AbstractDrivetrain drivetrain, double robotWidth,
		MotionLogger motionLogger) {
		running = false;
		Path.setRobotWidth(robotWidth);

		this.motionLogger = motionLogger;

		controller = new Timer();
		period = 5;

		this.drivetrain = drivetrain;
		kV = drivetrain.getKV();
		kK = drivetrain.getKK();
		kAcc = drivetrain.getKAcc();
		kS = drivetrain.getKS();
		kL = drivetrain.getKL();
		kAng = drivetrain.getKAng();
	}

	/**
	 * @param drivetrain - the drivetrain to use the AbstractDrivetrain methods from
	 * @param motionLogger - the MotionLogger from the AbstractDrivetrain
	 */
	public MotionController(AbstractDrivetrain drivetrain, MotionLogger motionLogger) {
		this(drivetrain, drivetrain.getRobotWidth(), motionLogger);
	}

	/**
	 * @param drivetrain - the drivetrain to use the AbstractDrivetrain methods from
	 */
	public MotionController(AbstractDrivetrain drivetrain) {
		this(drivetrain, drivetrain.getRobotWidth(), drivetrain.getMotionLogger());
	}

	/**
	 * Adds a path to the path queue
	 *
	 * @param paths - the paths to add to the queue
	 */
	public final void addPaths(Path... paths) {
		Collections.addAll(this.paths, paths);
	}

	/**
	 * Calculates the powers to send to the wheels
	 *
	 * @return a RobotPair with the powers and the time
	 */
	private synchronized RobotPair calculateSpeeds(RobotPair wheelPositions) {

		double leftPower = 0;
		double rightPower = 0;
		double steerPowerXTE;
		double steerPowerAngle;
		double centerPowerLag;

		if (running) {
			if (currentPath != null) {
				targetPathData = interpolate(wheelPositions);

				if (currentPath.isFinished()) {
					LinkedList<PathData> temp = currentPath.getPathData();
					currentPath = paths.pollFirst();
					if (currentPath != null) {
						double time = temp.getLast().getTime() - temp.getFirst().getTime();

						//Used to allow smooth transition between motions not making assumption that it finishes perfectly on time
						startingWheelPositions = new RobotPair(wheelPositions.getLeft(),
							wheelPositions.getRight(), time + startingWheelPositions.getTime());

						pdIterator = this.currentPath.getPathData().listIterator();
						pdPrevious = targetPathData = pdIterator.next();
						pdNext = pdIterator.next();

						staticPathData = null;
					} else {
						System.out.println("Done with motions! :)");

						//FIXME make this less messy
						staticPathData = new PathData(new State(wheelPositions.getLeft(), 0, 0),
							new State(wheelPositions.getRight(), 0, 0), new Pose(0, 0, 0), 0);
						targetPathData = staticPathData;
					}
				}
			}

			if (currentPath == null
				&& staticPathData == null) {  // if there is absolutely no more paths at the moment
				// says to not move

				//FIXME make it so that this scenario only runs when there is
				staticPathData = new PathData(new State(wheelPositions.getLeft(), 0, 0),
					new State(wheelPositions.getRight(), 0, 0), new Pose(0, 0, 0), 0);
				targetPathData = staticPathData;
			}

			// feed forward
			leftPower += ((kV * targetPathData.getLeftState().getVelocity())
				+ (kK * Math.signum(targetPathData.getLeftState().getVelocity())))
				+ (kAcc * targetPathData.getLeftState().getAcceleration());
			rightPower += ((kV * targetPathData.getRightState().getVelocity())
				+ (kK * Math.signum(targetPathData.getRightState().getVelocity())))
				+ (kAcc * targetPathData.getRightState().getAcceleration());
			// feed back
			steerPowerXTE = kS * errorVector.getXTrack();
			steerPowerAngle = kAng * errorVector.getAngle();
			centerPowerLag = kL * errorVector.getLag();

			double centerPower = ((leftPower + rightPower) / 2) + centerPowerLag;
			double steerPower = Math.max(-1,
				Math.min(1, ((rightPower - leftPower) / 2) + steerPowerXTE + steerPowerAngle));
			centerPower = Math
				.max(-1 + Math.abs(steerPower), Math.min(1 - Math.abs(steerPower), centerPower));
			return new RobotPair(centerPower - steerPower, centerPower + steerPower,
				wheelPositions.getTime());
		}
		return new RobotPair(0, 0, wheelPositions.getTime());
	}

	/**
	 * Finds the target x, y, angle, velocityLeft, and velocityRight
	 *
	 * @return a new MotionData with the interpolated data
	 */
	private PathData interpolate(RobotPair wheelPositions) {
		double currentTime = wheelPositions.getTime() - startingWheelPositions.getTime();
		while (currentTime > pdNext.getTime()) {
			if (pdIterator.hasNext()) {
				pdPrevious = pdNext;
				pdNext = pdIterator.next();
			} else {
				pdPrevious = pdNext;
				currentPath.setFinished(true);
				return pdNext;
			}
		}

		double timePrevious = pdPrevious.getTime();
		double timeNext = pdNext.getTime();
		double dTime = timeNext - timePrevious;
		double rctn =
			(timeNext - currentTime) / dTime; // Ratio of the current time to the next pose time
		double rltc =
			(currentTime - timePrevious) / dTime; // Ratio of the previous time to the current pose
		// time

		double lengthLeft = ((pdPrevious.getLeftState().getLength()) * rctn)
			+ ((pdNext.getLeftState().getLength()) * rltc);
		double lengthRight = ((pdPrevious.getRightState().getLength()) * rctn)
			+ ((pdNext.getRightState().getLength()) * rltc);

		// Current pose is made from the weighted average of the x, y, and angle values
		double x =
			(pdPrevious.getCenterPose().getX() * rctn) + (pdNext.getCenterPose().getX() * rltc);
		double y =
			(pdPrevious.getCenterPose().getY() * rctn) + (pdNext.getCenterPose().getY() * rltc);
		double angle =
			(pdPrevious.getCenterPose().getAngle() * rctn) + (pdNext.getCenterPose().getAngle()
				* rltc);

		State left = new State(lengthLeft, pdNext.getLeftState().getVelocity(),
			pdNext.getLeftState().getAcceleration());
		State right = new State(lengthRight, pdNext.getRightState().getVelocity(),
			pdNext.getRightState().getAcceleration());
		Pose centerPose = new Pose(x, y, angle);
		return new PathData(left, right, centerPose, currentTime);
	}

	/**
	 * Removes all queued motions
	 */
	public synchronized final void clearMotions() {
		paths.clear();
	}

	/**
	 * Starts the queue of motions
	 */
	public synchronized final void enableScheduler() {
		if (!running) {
			staticPathData = new PathData(new State(drivetrain.getWheelPositions().getLeft(), 0, 0),
				new State(drivetrain.getWheelPositions().getRight(), 0, 0), new Pose(0, 0, 0), 0);
			Path newPath = paths.poll();
			if (newPath != null) {
				currentPath = newPath;
				running = true;
				actualPosition = currentPath.getPathData().get(0).getCenterPose();
				previousLengths = startingWheelPositions = drivetrain.getWheelPositions();
				pdIterator = currentPath.getPathData().listIterator();
				pdPrevious = targetPathData = pdIterator.next();
				pdNext = pdIterator.next();

				currentTimerTask = new MotionTask();
				controller.schedule(currentTimerTask, 0L, (long) period);
			} else {
				running = false;
			}
		}
	}

	/**
	 * @return Whether or not the queue has ended
	 */
	public final boolean isFinished() {
		return currentPath == null;
	}

	/**
	 * @return Whether or not a motion is running
	 */
	public final boolean isRunning() {
		return running;
	}

	/**
	 * Pauses the motions,
	 */
	public synchronized final void stopScheduler() {
		if (running) {
			running = false;
			currentTimerTask.cancel();
			controller.purge();
			currentPath = null;
			drivetrain.setSpeeds(0, 0);
		}
	}

	/**
	 * Updates where the robot thinks it is, based off of the encoder lengths
	 */
	private void updateActualPosition(RobotPair wheelPositions) {
		double arcLeft = wheelPositions.getLeft() - previousLengths.getLeft();
		double arcRight = wheelPositions.getRight() - previousLengths.getRight();
		double dAngle = (arcRight - arcLeft) / Path.getRobotWidth();
		double arcCenter = (arcRight + arcLeft) / 2;
		double dX;
		double dY;
		if (Math.abs(dAngle) < 0.01) {
			dX = arcCenter * StrictMath.cos(actualPosition.getAngle());
			dY = arcCenter * StrictMath.sin(actualPosition.getAngle());
		} else {
			dX = arcCenter * (
				((StrictMath.sin(dAngle) * StrictMath.cos(actualPosition.getAngle())) / dAngle)
					- (((StrictMath.cos(dAngle) - 1) * StrictMath.sin(actualPosition.getAngle()))
					/ dAngle));
			dY = arcCenter * (
				((StrictMath.sin(dAngle) * StrictMath.sin(actualPosition.getAngle())) / dAngle)
					- (((StrictMath.cos(dAngle) - 1) * StrictMath.cos(actualPosition.getAngle()))
					/ dAngle));
		}

		actualPosition = actualPosition.offset(dX, dY, dAngle);
		previousLengths = wheelPositions;
	}

	/**
	 * Finds the current lag and cross track ErrorVector
	 */
	private void findCurrentError() {
		Pose targetPose = targetPathData.getCenterPose();
		Pose actualPose = actualPosition;
		double dX = targetPose.getX() - actualPose.getX();
		double dY = targetPose.getY() - actualPose.getY();
		double angle = targetPose.getAngle();
		// error in direction facing
		double lagError = (dX * StrictMath.cos(angle)) + (dY * StrictMath.sin(angle));
		// error perpendicular to direction facing
		double crossTrackError = (-dX * StrictMath.sin(angle)) + (dY * StrictMath.cos(angle));
		// the error of the current angle
		double angleError = targetPose.getAngle() - actualPose.getAngle();
		if (angleError > Math.PI) {
			angleError -= 2 * Math.PI;
		} else if (angleError < -Math.PI) {
			angleError += 2 * Math.PI;
		}
		errorVector = new ErrorVector(lagError, crossTrackError, angleError);
	}

	@Override
	public final String toString() {
		return "MotionController{" +
			"paths=" + paths +
			", controller=" + controller +
			", running=" + running +
			", period=" + period +
			", currentPath=" + currentPath +
			", staticPathData=" + staticPathData +
			", drivetrain=" + drivetrain +
			", kV=" + kV +
			", kK=" + kK +
			", kAcc=" + kAcc +
			", kS=" + kS +
			", kL=" + kL +
			", kAng=" + kAng +
			", actualPosition=" + actualPosition +
			", targetPathData=" + targetPathData +
			", previousLengths=" + previousLengths +
			", startingWheelPositions=" + startingWheelPositions +
			", pdIterator=" + pdIterator +
			", pdPrevious=" + pdPrevious +
			", pdNext=" + pdNext +
			", errorVector=" + errorVector +
			", motionLogger=" + motionLogger +
			", powers=" + powers +
			'}';
	}

	/**
	 * 1 Runs the calculations with a TimerTask
	 *
	 * @author Russell Newton, WaltonRobotics
	 */
	private class MotionTask extends TimerTask {

		@Override
		public final void run() {
			if (currentPath != null) {
				RobotPair wheelPositions = drivetrain.getWheelPositions();
				updateActualPosition(wheelPositions);
				findCurrentError();
				powers = calculateSpeeds(wheelPositions);
				drivetrain.setSpeeds(powers.getLeft(), powers.getRight());
				motionLogger.addMotionData(
					new MotionData(actualPosition, targetPathData.getCenterPose(), errorVector,
						powers));
			}
		}

	}
}
