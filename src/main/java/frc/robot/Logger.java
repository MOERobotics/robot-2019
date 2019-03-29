package frc.robot;

public class Logger <valueType>{
    valueType oldValue = null;

    public void printIfChanged(String subject, valueType newValue) {
        if (oldValue == null || !oldValue.equals(newValue)) {
            System.out.printf(
                    "[%d] %s set to %s\n",
                    System.currentTimeMillis(),
                    subject + " power",
                    newValue
            );
            System.out.flush();
            oldValue = newValue;
        }
    }
}
