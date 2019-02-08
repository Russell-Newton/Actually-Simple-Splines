package frc.team2974.robot;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.Config.Path;
import frc.team2974.robot.subsystems.Drivetrain;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.Line;
import org.waltonrobotics.motion.PointTurn;
import org.waltonrobotics.motion.Spline;
import org.waltonrobotics.util.Controls;
import org.waltonrobotics.util.EncoderConfig;
import org.waltonrobotics.util.RobotConfig;
import org.waltonrobotics.util.TalonConfig;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {

  public static Drivetrain drivetrain;

  RobotConfig robotConfig = new RobotConfig("Test") {
    @Override
    public EncoderConfig getRightEncoderConfig() {
      return null;
    }

    @Override
    public EncoderConfig getLeftEncoderConfig() {
      return null;
    }

    @Override
    public TalonConfig getLeftTalonConfig() {
      return null;
    }

    @Override
    public TalonConfig getRightTalonConfig() {
      return null;
    }

    @Override
    public Controls getRightJoystickConfig() {
      return null;
    }

    @Override
    public Controls getLeftJoystickConfig() {
      return null;
    }

    @Override
    public double getMaxAcceleration() {
      return 1;
    }

    @Override
    public double getKV() {
      return 1;
    }

    @Override
    public double getKAcc() {
      return 0;
    }

    @Override
    public double getKK() {
      return 0;
    }

    @Override
    public double getKS() {
      return 0;
    }

    @Override
    public double getKAng() {
      return 0;
    }

    @Override
    public double getKL() {
      return 0;
    }

    @Override
    public double getILag() {
      return 0;
    }

    @Override
    public double getIAng() {
      return 0;
    }

    @Override
    public double getRobotWidth() {
      return 0;
    }

    @Override
    public double getRobotLength() {
      return 0;
    }

    @Override
    public boolean isCurrentRobot() {
      return true;
    }
  };

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain = new Drivetrain(robotConfig);

    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();
    drivetrain.getMotionLogger().writeMotionDataCSV();

    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.startControllerMotion(Pose.ZERO);
    drivetrain.getMotionLogger().initialize();

    Line starightLine = new Line(Path.VELOCITY_MAX, Path.ACCELERATION_MAX, 0, 0, false, drivetrain.getActualPosition(),
        1);

    PointTurn pointTurn = new PointTurn(Path.VELOCITY_MAX, Path.ACCELERATION_MAX, drivetrain.getActualPosition(), 180);

    Spline spline = new Spline(Path.VELOCITY_MAX, Path.ACCELERATION_MAX, 0, 0, false, drivetrain.getActualPosition(),
        Pose.ZERO);

    drivetrain.addControllerMotions(starightLine, pointTurn, spline);

    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();

    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during operator control
   */
  @Override
  public void teleopPeriodic() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode
   */
  @Override
  public void testPeriodic() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  /**
   * Put things in here you want to update for SmartDashboard.
   */
  private void updateSmartDashboard() {
    SmartDashboard.putString("Robot position", String.valueOf(drivetrain.getActualPosition()));

  }

}
