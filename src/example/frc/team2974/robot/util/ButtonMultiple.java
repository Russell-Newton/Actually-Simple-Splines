package frc.team2974.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * This button is used when you have multiple alternate buttons for the same purpose.
 */
public class ButtonMultiple extends Button {

  private final GenericHID joystick;
  private final int[] buttonNumbers;

  /**
   * Creates a multi-button for triggering commands.
   *
   * @param joystick the GenericHID object that has the button
   * @param buttonNumbers the button numbers
   */
  public ButtonMultiple(GenericHID joystick, int... buttonNumbers) {
    this.joystick = joystick;
    this.buttonNumbers = buttonNumbers;
  }

  /**
   * Gets the value of the joystick buttons.
   *
   * @return true if any of the buttons are pressed, false otherwise.
   */
  @Override
  public boolean get() {
    for (int buttonNumber : buttonNumbers) {
      if (joystick.getRawButton(buttonNumber)) {
        return true;
      }
    }
    return false;
  }
}
