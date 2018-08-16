package org.waltonrobotics.controller;

import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.MotionLogger;
import org.waltonrobotics.motion.Path;

import java.util.*;
import java.util.concurrent.LinkedBlockingDeque;

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
    private final Queue<Path> paths = new LinkedBlockingDeque<>();
    private final int period;
    private final MotionLogger motionLogger;
    private final double iAng;
    private final double iLag;
    private final Timer controller;
    private boolean running;
    private Path currentPath;
    private PathData staticPathData;
    private Pose actualPosition;
    private PathData targetPathData;
    private RobotPair previousLengths;
    private double pathStartTime;
    private ListIterator<PathData> pdIterator;
    private PathData pdPrevious;
    private PathData pdNext;
    private ErrorVector errorVector;
    private RobotPair powers;
    private TimerTask currentTimerTask;
    private MotionState currentMotionState = MotionState.WAITING;
    private double integratedLagError;
    private double integratedAngleError;
    private int pathNumber;

    /**
     * @param drivetrain   - the drivetrain to use the AbstractDrivetrain methods from
     * @param robotWidth   - the robot width from the outside of the wheels
     * @param motionLogger - the MotionLogger from the AbstractDrivetrain
     */
    public MotionController(AbstractDrivetrain drivetrain, double robotWidth,
                            MotionLogger motionLogger) {
        running = false;
        Path.setRobotWidth(robotWidth);

        this.motionLogger = motionLogger;

        controller = new Timer();
        period = 5;

        RobotPair wheelPositions = drivetrain.getWheelPositions();
        staticPathData = new PathData(new State(wheelPositions.getLeft(), 0, 0),
                new State(wheelPositions.getRight(), 0, 0), new Pose(0, 0, 0), 0, true);

        this.drivetrain = drivetrain;
        kV = drivetrain.getKV();
        kK = drivetrain.getKK();
        kAcc = drivetrain.getKAcc();
        kS = drivetrain.getKS();
        kL = drivetrain.getKL();
        kAng = drivetrain.getKAng();
        iLag = drivetrain.getILag();
        iAng = drivetrain.getIAng();

        pathNumber = 0;
    }

    /**
     * @param drivetrain   - the drivetrain to use the AbstractDrivetrain methods from
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
        if (running) {
            double leftPower = 0;
            double rightPower = 0;

            if (currentPath != null) {
                targetPathData = interpolate(wheelPositions);

                if (currentPath.isFinished()) {
                    System.out.println("Current path is finished");
                    Deque<PathData> temp = currentPath.getPathData();
                    currentPath = paths.poll();

                    integratedLagError = 0;
                    integratedAngleError = 0;

                    if (currentPath != null) {
                        System.out.println("Getting new path");
                        double time = temp.getLast().getTime() - temp.getFirst().getTime();

                        //Used to allow smooth transition between motions not making assumption that it finishes perfectly on time
                        pathStartTime = time + pathStartTime;

                        pdIterator = currentPath.getPathData().listIterator();
                        pdPrevious = targetPathData = pdIterator.next();
                        pdNext = pdIterator.next();

                        targetPathData = interpolate(wheelPositions);
                        currentMotionState = MotionState.MOVING;
                        pathNumber += 1;
                    } else {
                        System.out.println("Done with motions! :)");

                        staticPathData = new PathData(new State(wheelPositions.getLeft(), 0, 0),
                                new State(wheelPositions.getRight(), 0, 0),
                                targetPathData.getCenterPose(), wheelPositions.getTime(),
                                targetPathData.isBackwards());
                        targetPathData = staticPathData;
                        currentMotionState = MotionState.FINISHING;
                    }
                }
            } else {
                // if there is absolutely no more paths at the moment says to not move

                currentPath = paths.poll();
                if (currentPath != null) {
                    System.out.println("Getting initial path");
//					actualPosition = currentPath.getPathData().get(0).getCenterPose();
                    pathStartTime = wheelPositions.getTime();
                    pdIterator = currentPath.getPathData().listIterator();
                    pdPrevious = targetPathData = pdIterator.next();
                    pdNext = pdIterator.next();

                    currentMotionState = MotionState.MOVING;
                    targetPathData = interpolate(wheelPositions);

                    integratedLagError = 0;
                    integratedAngleError = 0;

                    pathNumber += 1;
                } else {
//					System.out.println("No initial path not moving");
                    targetPathData = staticPathData;
                }
            }
            actualPosition = updateActualPosition(wheelPositions, previousLengths, actualPosition);
            previousLengths = wheelPositions;

            errorVector = findCurrentError(targetPathData, actualPosition);

            double centerPower = 0;
            double steerPower = 0;
            if (currentMotionState == MotionState.MOVING) {
                // feed forward

                leftPower += ((kV * targetPathData.getLeftState().getVelocity())
                        + (kK * Math.signum(targetPathData.getLeftState().getVelocity())))
                        + (kAcc * targetPathData.getLeftState().getAcceleration());
                rightPower += ((kV * targetPathData.getRightState().getVelocity())
                        + (kK * Math.signum(targetPathData.getRightState().getVelocity())))
                        + (kAcc * targetPathData.getRightState().getAcceleration());
                // feed back
                double steerPowerXTE = kS * errorVector.getXTrack();
                double steerPowerAngle = kAng * errorVector.getAngle();
                double centerPowerLag = kL * errorVector.getLag();

                centerPower = ((leftPower + rightPower) / 2.0) + centerPowerLag;
                steerPower = Math.max(-1,
                        Math.min(1, ((rightPower - leftPower) / 2) + steerPowerXTE + steerPowerAngle));
                centerPower = Math
                        .max(-1 + Math.abs(steerPower),
                                Math.min(1 - Math.abs(steerPower), centerPower));
            }
            if ((currentMotionState == MotionState.FINISHING) || isClose(1)) {
//          to give the extra oomph when finished the path but there is a little bit more to do//FIXME left, right powers somehow manage to be greater than 1

                if ((wheelPositions.getTime() - staticPathData.getTime()) >= 2) {
                    currentMotionState = MotionState.WAITING;
                }

                integratedLagError += iLag * errorVector.getLag();
                integratedAngleError += iAng * errorVector.getAngle();

                integratedAngleError = Math.max(Math.min(0.5, integratedAngleError), -0.5);
                integratedLagError = Math.max(Math.min(0.5, integratedLagError), -0.5);

                steerPower += integratedAngleError;
                centerPower += integratedLagError;


            }

            if (currentMotionState == MotionState.WAITING) {
                steerPower = 0;
                centerPower = 0;
            }

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
        double currentTime = wheelPositions.getTime() - pathStartTime;
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
                (currentTime - timePrevious)
                        / dTime; // Ratio of the previous time to the current pose
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
        return new PathData(left, right, centerPose, currentTime, currentPath.isBackwards());
    }

    /**
     * Removes all queued motions
     */
    public final synchronized void clearMotions() {
        paths.clear();
    }

    /**
     * Starts the queue of motions
     */
    public final synchronized void enableScheduler() {
        if (!running) {
            System.out.println("Enabling scheduler");
            System.out.println(actualPosition);
//			actualPosition = starting;
            if (actualPosition == null) {
                System.out
                        .println("Starting position might not have been set setting actual position to 0,0,0");
                actualPosition = new Pose(0, 0, 0);
            }

            previousLengths = drivetrain.getWheelPositions();

            staticPathData = new PathData(
                    new State(drivetrain.getWheelPositions().getLeft(), 0, 0),
                    new State(drivetrain.getWheelPositions().getRight(), 0, 0),
                    actualPosition,
                    0, true);

            targetPathData = staticPathData;

            currentTimerTask = new MotionTask();
            controller.schedule(currentTimerTask, 0L, period);
            currentMotionState = MotionState.WAITING;
            running = true;
        }
    }

    /**
     * @return Whether or not the queue has ended
     */
    public final boolean isFinished() {
        return (currentPath == null) && (currentMotionState == MotionState.FINISHING);
    }

    /**
     * @return Whether or not a motion is running
     */
    public final boolean isRunning() {
        return running;
    }

    /**
     * @return Percent of the current Path that the robot is at, based off of the time
     */
    public double getPercentDone(Path pathToUse) {
        if (currentPath.equals(pathToUse)) {
            double currentTime = drivetrain.getWheelPositions().getTime() - pathStartTime;
            double endTime = currentPath.getPathData().getLast().getTime();
            return currentTime / endTime;
        }
        return -1;
    }

    /**
     * Pauses the motions,
     */
    public final synchronized void stopScheduler() {
        if (running) {
            System.out.println("Disabling scheduler");
            running = false;
            currentTimerTask.cancel();
            controller.purge();
            currentPath = null;
            drivetrain.setSpeeds(0, 0);
            pathNumber = 0;
        }
    }

    /**
     * Updates where the robot thinks it is, based off of the encoder lengths
     */
    public Pose updateActualPosition(RobotPair wheelPositions, RobotPair previousWheelPositions,
                                     Pose estimatedActualPosition) {
        double arcLeft = wheelPositions.getLeft() - previousWheelPositions.getLeft();
        double arcRight = wheelPositions.getRight() - previousWheelPositions.getRight();
        double dAngle = (arcRight - arcLeft) / Path.getRobotWidth();
        double arcCenter = (arcRight + arcLeft) / 2;
        double dX;
        double dY;
        if (Math.abs(dAngle) < 0.01) {
            dX = arcCenter * StrictMath.cos(estimatedActualPosition.getAngle());
            dY = arcCenter * StrictMath.sin(estimatedActualPosition.getAngle());
        } else {
            dX = arcCenter * (
                    ((StrictMath.sin(dAngle) * StrictMath.cos(estimatedActualPosition.getAngle())) / dAngle)
                            - (
                            ((StrictMath.cos(dAngle) - 1) * StrictMath.sin(estimatedActualPosition.getAngle()))
                                    / dAngle));
            dY = arcCenter * (
                    ((StrictMath.sin(dAngle) * StrictMath.sin(estimatedActualPosition.getAngle())) / dAngle)
                            - (
                            ((StrictMath.cos(dAngle) - 1) * StrictMath.cos(estimatedActualPosition.getAngle()))
                                    / dAngle));
        }

        estimatedActualPosition = estimatedActualPosition.offset(dX, dY, dAngle);

        return estimatedActualPosition;
    }

    /**
     * Finds the current lag and cross track ErrorVector
     */
    private ErrorVector findCurrentError(PathData targetPathData, Pose actualPose) {
        Pose targetPose = targetPathData.getCenterPose();
        double dX = targetPose.getX() - actualPose.getX();
        double dY = targetPose.getY() - actualPose.getY();
        double angle = targetPose.getAngle();
        // error in direction facing
        double lagError = (dX * StrictMath.cos(angle)) + (dY * StrictMath.sin(angle));
        // error perpendicular to direction facing

        double crossTrackError = (-dX * StrictMath.sin(angle)) + (dY * StrictMath.cos(angle));
        // the error of the current angle
        double angleError = targetPose.getAngle() - actualPose.getAngle();

        if (targetPathData.isBackwards()) {
            crossTrackError *= -1;
        }

        if (angleError > Math.PI) {
            angleError -= 2 * Math.PI;
        } else if (angleError < -Math.PI) {
            angleError += 2 * Math.PI;
        }
        return new ErrorVector(lagError, crossTrackError, angleError);
    }

    public boolean isClose(double closeTime) {
        return (currentPath != null) && (targetPathData != null)
                && (((currentPath.getPathData().getLast().getTime() + pathStartTime) - targetPathData
                .getTime())
                <= closeTime);
    }

    @Override
    public String toString() {
        return "MotionController{" +
                "drivetrain=" + drivetrain +
                ", kV=" + kV +
                ", kK=" + kK +
                ", kAcc=" + kAcc +
                ", kS=" + kS +
                ", kL=" + kL +
                ", kAng=" + kAng +
                ", paths=" + paths +
                ", period=" + period +
                ", motionLogger=" + motionLogger +
                ", iAng=" + iAng +
                ", iLag=" + iLag +
                ", controller=" + controller +
                ", running=" + running +
                ", currentPath=" + currentPath +
                ", staticPathData=" + staticPathData +
                ", actualPosition=" + actualPosition +
                ", targetPathData=" + targetPathData +
                ", previousLengths=" + previousLengths +
                ", pathStartTime=" + pathStartTime +
                ", pdIterator=" + pdIterator +
                ", pdPrevious=" + pdPrevious +
                ", pdNext=" + pdNext +
                ", errorVector=" + errorVector +
                ", powers=" + powers +
                ", currentTimerTask=" + currentTimerTask +
                ", currentMotionState=" + currentMotionState +
                ", integratedLagError=" + integratedLagError +
                ", integratedAngleError=" + integratedAngleError +
                ", pathNumber=" + pathNumber +
                '}';
    }

    public void setStartPosition(Pose startingPosition) {

        actualPosition = startingPosition;
    }

    /**
     * 1 Runs the calculations with a TimerTask
     *
     * @author Russell Newton, WaltonRobotics
     */
    private class MotionTask extends TimerTask {

        @Override
        public final void run() {
            RobotPair wheelPositions = drivetrain.getWheelPositions();

            powers = calculateSpeeds(wheelPositions);

            drivetrain.setSpeeds(powers.getLeft(), powers.getRight());

            motionLogger.addMotionData(
                    new MotionData(actualPosition, targetPathData.getCenterPose(), errorVector,
                            powers, pathNumber, currentMotionState));
        }
    }
}
