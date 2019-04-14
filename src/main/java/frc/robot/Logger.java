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

    }

    public static class Joystick {
        public GenericHID joystick;
        public String name;
        int numButtons;
        public boolean[] lastState;

        public Joystick(String name, GenericHID joystick) {
            this.joystick = joystick;
            this.name = name;
            this.numButtons = joystick.getButtonCount();
            this.lastState = new boolean[numButtons];
        }

        public void printIfChanged() {
            for (int i = 1; i < numButtons; i++) {
                int rawButtonNumber = i + 1;
                boolean isButtonPressed = joystick.getRawButton(rawButtonNumber);
                if (isButtonPressed != lastState[i]) {
                    String action = (isButtonPressed ? "pressed" : "released");
                    printLogMessage(name + " button " + rawButtonNumber, action);
                    lastState[i] = isButtonPressed;
                }
            }
        }
    }
}
