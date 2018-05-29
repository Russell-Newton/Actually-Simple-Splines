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
import org.waltonrobotics.motion.SavePath;

public final class RunningSavedPath extends SimpleMotion {


	private RunningSavedPath(List<Moment> movement) {
		super(SavePath.createPathData(movement));
	}

	private RunningSavedPath(Moment... movement) {
		super(SavePath.createPathData(movement));
	}


	public static RunningSavedPath loadSavedPath(String filePath) {
		try (BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(filePath)))) {

			List<Moment> moments = bufferedReader.lines().map(
				line -> {
					double[] data = Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
					return new Moment(data[0], data[1], data[2], data[3], data[4], data[5]);
				}
			).collect(Collectors.toList());

			return new RunningSavedPath(moments);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		return new RunningSavedPath();
	}

	public static RunningSavedPath loadSavedPath(LearnPath learnedPath) {
		return new RunningSavedPath(learnedPath.getMoments());
	}

}


