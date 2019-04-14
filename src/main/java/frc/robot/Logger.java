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

    //in  =   -1  <->   1
    //out = -100  <-> 100

    public static int doubleToPercent(Double value) {

        return (int)(value * 100);


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

    public static class Joystick {
        GenericHID joystick;
        String name;
        int numButtons;
        boolean[] lastState;

        public Joystick(String name, GenericHID joystick) {
            this.joystick = joystick;
            this.name = name;
            this.numButtons = joystick.getButtonCount();
            this.lastState = new boolean[numButtons];
        }

        public void printIfChanged() {

            for (int i = 0; i < numButtons; i++) {
                int rawButtonNumber = i+1;
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
