package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class Logger <valueType>{
    valueType oldValue = null;

    public void printIfChanged(String subject, valueType newValue) {
        if (oldValue == null || !oldValue.equals(newValue)) {
            printLogMessage(subject+" power", newValue.toString());
            oldValue = newValue;
        }
    }
    public static void printLogMessage(String subject, String value) {

        System.out.printf(
                "[%d] %s set to %s\n",
                System.currentTimeMillis(),
                subject,
                value
        );
        System.out.flush();
    }

    public static void printJoystickIfChanged(String joystickName, GenericHID joystick) {

        int num_buttons = joystick.getButtonCount();

        for (int i = 0; i < num_buttons; i++) {
            if (joystick.getRawButtonPressed(i)) {
                printLogMessage(joystickName + " button "+i, "pressed");
            }
            if (joystick.getRawButtonReleased(i)) {
                printLogMessage(joystickName + " button "+i, "released");
            }
        }

    }
}
