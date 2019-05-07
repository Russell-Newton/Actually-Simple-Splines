package org.waltonrobotics.metadata.swerve;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.motion.Path;
import org.waltonrobotics.util.Vector;

/**
 * @author Russell Newton
 **/
public class SwerveState {

  private final double translationalVelocity;
  private final double centerAngle;
  private final List<SwervePair> swervePairs;
  private final List<Vector> relativePairPositions;

  public SwerveState(double translationalVelocity, double centerAngle,
      List<SwervePair> swervePairs) {
    this.translationalVelocity = translationalVelocity;
    this.centerAngle = centerAngle;
    this.swervePairs = swervePairs;
    relativePairPositions = calculateRelativePairPositions();
  }

  public SwerveState(double translationalVelocity, double centerAngle) {
    this.translationalVelocity = translationalVelocity;
    this.centerAngle = centerAngle;
    swervePairs = new LinkedList<>();
    swervePairs.add(new SwervePair());
    swervePairs.add(new SwervePair());
    swervePairs.add(new SwervePair());
    swervePairs.add(new SwervePair());

    relativePairPositions = calculateRelativePairPositions();
  }

  public SwerveState() {
    this(0, 0);
  }

  public double getTranslationalVelocity() {
    return translationalVelocity;
  }

  public double getCenterAngle() {
    return centerAngle;
  }

  public List<SwervePair> getSwervePairs() {
    return swervePairs;
  }

  public List<Vector> getRelativePairPositions() {
    return relativePairPositions;
  }

  @Override
  public String toString() {
    return "SwerveState{" +
        "translationalVelocity=" + translationalVelocity +
        ", centerAngle=" + centerAngle +
        ", swervePairs=" + swervePairsToString()+
        "}";
  }

  private String swervePairsToString() {
    StringBuilder sb = new StringBuilder("{");
    for(SwervePair swervePair : swervePairs) {
      sb.append(swervePair.toString());
      sb.append(", ");
    }
    return sb.substring(0, sb.length() - 2) + "}";
  }

  private List<Vector> calculateRelativePairPositions() {
    double halfLength = Path.getRobotLength() / 2;
    double halfWidth = Path.getRobotWidth() / 2;

    List<Vector> relativePairPositions = new LinkedList<>();
    Collections.addAll(relativePairPositions,
        new Vector(halfWidth, halfLength), new Vector(-halfWidth, halfLength),
        new Vector(-halfWidth, -halfLength), new Vector(halfWidth, -halfLength));

    for (int i = 0; i < 4; i++) {
      relativePairPositions.set(i, relativePairPositions.get(i).rotate(centerAngle));
    }

    return relativePairPositions;
  }
}
