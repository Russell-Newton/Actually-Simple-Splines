package frc.team2974.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * get() returns true only the first "frame" it is pressed.
 */
public class ButtonOnce extends JoystickButton {

  private boolean down;

  public ButtonOnce(GenericHID hid, int channel) {
    super(hid, channel);
    down = false;
  }

  @Override
  public boolean get() {
    if (super.get()) {
      if (!down) {
        down = true;
        return true;
      }
    } else {
      down = false;
    }

    return false;
  }
}
