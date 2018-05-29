package org.waltonrobotics.motion;

import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import org.waltonrobotics.controller.Moment;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.State;

public final class SavePath extends Path {

	private SavePath(double vCruise, double aMax,
		Collection<PathData> pathData) {
		super(vCruise, aMax, false, pathData.stream().map(PathData::getCenterPose).collect(Collectors.toList()));

		getPathData().clear();
		getPathData().addAll(pathData);
	}

	public static Path createPathData(Moment... moments) {
		return createPathData(Arrays.asList(moments));
	}

	public static Path createPathData(List<Moment> moments) {
		List<PathData> pathData = new LinkedList<>();

		double maxVelocity = 0;
		double maxAcceleration = 0;

		if (moments.size() >= 1) {
			//First data point has 0 velocity and 0 acceleration
			Moment previous = moments.get(0);
			pathData.add(new PathData(previous.getActualPosition()));

			if (moments.size() >= 2) {
				// Second data point has velocity but 0 acceleration
				Moment current = moments.get(1);
				double deltaTime = current.getTime() - previous.getTime();
				double lVelocity = (current.getLeftLength() - previous.getLeftLength()) / deltaTime;
				double rVelocity = (current.getRightLength() - previous.getRightLength()) / deltaTime;

				pathData.add(new PathData(
					new State(
						current.getLeftLength(),
						lVelocity,
						0
					),
					new State(
						current.getRightLength(),
						rVelocity,
						0
					),
					current.getActualPosition(),
					current.getTime()));

				maxVelocity = Math.max(Math.abs(lVelocity), Math.abs(rVelocity));

				for (int i = 2; i < moments.size(); i++) {
					PathData previousState = pathData.get(i - 1);
					current = moments.get(i);

					deltaTime = current.getTime() - previousState.getTime();

					lVelocity = (current.getLeftLength() - previousState.getLeftState().getLength()) / deltaTime;
					rVelocity = (current.getRightLength() - previousState.getRightState().getLength()) / deltaTime;

					double rAcceleration = (lVelocity - previousState.getLeftState().getVelocity()) / deltaTime;
					double lAcceleration = (rVelocity - previousState.getRightState().getVelocity()) / deltaTime;

					maxVelocity = Math.max(Math.max(Math.abs(lVelocity), Math.abs(rVelocity)), maxVelocity);
					maxAcceleration = Math
						.max(Math.max(Math.abs(lAcceleration), Math.abs(rAcceleration)), maxAcceleration);

					pathData.add(new PathData(
						new State(current.getLeftLength(), lVelocity, lAcceleration),
						new State(current.getRightLength(), rVelocity, rAcceleration),
						current.getActualPosition(), current.getTime()
					));
				}
			}
		}
		return new SavePath(maxVelocity, maxAcceleration, pathData);
	}

	@Override
	public void createPath() {
	}
}
