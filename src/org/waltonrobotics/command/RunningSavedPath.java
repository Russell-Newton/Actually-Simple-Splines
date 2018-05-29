package org.waltonrobotics.command;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.waltonrobotics.controller.Moment;
import org.waltonrobotics.controller.RobotPair;
import org.waltonrobotics.motion.SavePath;

public final class RunningSavedPath extends SimpleMotion {


	private RunningSavedPath(List<Moment> movement) {
		super(SavePath.createPathData(movement));
	}

	private RunningSavedPath(Moment... movement) {
		this(Arrays.asList(movement));
	}


	public static RunningSavedPath loadSavedPath(String filePath) {
		return loadSavedPath(filePath, 1);
	}

	public static RunningSavedPath loadSavedPath(String filePath, double speedIncrease) {
		try (BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(filePath)))) {

			List<Moment> moments = bufferedReader.lines().map(
				line -> {
					double[] data = Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
					return new Moment(data[0], data[1], data[2], data[3], data[4], data[5]);
				}
			).collect(Collectors.toList());

			return new RunningSavedPath(scaleTime(moments, speedIncrease));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		return new RunningSavedPath();
	}

	/**
	 * Will scale time to be short the bigger the number. E.i. passing in 3 means that time will go 3 time faster
	 * meaning that the robot will complete the motion faster.
	 */
	private static List<Moment> scaleTime(List<Moment> moments, double inverseScale) {
		return moments.stream().map(moment ->
			new Moment(
				moment.getActualPosition(),
				new RobotPair(
					moment.getLeftLength(),
					moment.getRightLength(),
					moment.getTime() / inverseScale)
			)
		).collect(Collectors.toList());
	}

	public static RunningSavedPath loadSavedPath(LearnPath learnedPath) {
		return loadSavedPath(learnedPath, 1);
	}

	public static RunningSavedPath loadSavedPath(LearnPath learnedPath, double speedIncrease) {
		return new RunningSavedPath(scaleTime(learnedPath.getMoments(), speedIncrease));
	}

}


