package kevin_nonsense;

import java.io.OutputStream;
import java.io.Writer;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Logger {
    private static Map<String,LoggingData> logging_data;
    public static class LoggingData {
        public Long lastAlert;
        public Comparable lastValue;
    }

    public static synchronized LoggingData getLogData (String key) {
        if (logging_data == null) {
            logging_data = new HashMap<>();
        }
        if (!logging_data.containsKey(key)) {
            logging_data.put(key,new LoggingData());
        }
        return logging_data.get(key);
    }


    private Long logInterval = null;
    private boolean suppressIfEqual = false;
    public final String key;
    public final OutputStream outputLocation;

    public Logger(String key, OutputStream outputLocation) {
        this.key = key;
        this.outputLocation = outputLocation;
    }

    public Logger suppressIfEqual() {
        suppressIfEqual = true;
        return this;
    }

    public Logger alertIfEqual() {
        suppressIfEqual = false;
        return this;
    }

    public Logger minLogInterval(Long logInterval) {
        this.logInterval = logInterval;
        return this;
    }

    public Logger logOnce() {
        this.logInterval = (long) Integer.MAX_VALUE;
        return this;
    }

    public Logger unsetLogInterval() {
        return minLogInterval(null);
    }

    public LoggingData getLogData() {
        return getLogData(key);
    }

    public Logger logMessage(Comparable value, String message) {
        LoggingData data = getLogData();
        long now = System.currentTimeMillis();

        if (
            (
                data.lastAlert == null ||
                logInterval == null ||
                now >= data.lastAlert + logInterval
            ) && (
                !suppressIfEqual ||
                value == null ||
                value.compareTo(data.lastValue) != 0
            )
        ) {
            data.lastAlert = now;
            if (value != null) data.lastValue = value;
            try {
                outputLocation.write(String.format(
                        "%s: %s\n",
                        key,
                        message
                ).getBytes());
            } catch (Exception ignored) {}
        }
        return this;
    }

    public Logger logMessage(String message) {
        return logMessage(null,message);
    }

    public Logger logFormat(Comparable value, String format, Object... args) {
        String message = String.format(format,args);
        return logMessage(value, message);
    }

    public Logger logFormat(String format, Object... args) {
        return logFormat(null,format,args);
    }

    public Logger logValue(Comparable value) {
        return logFormat(value, "Value: %s", value.toString());
    }

    public Logger logTrace(Comparable value) {
        return logMessage(
            value,
            Arrays.toString(Thread.currentThread().getStackTrace())
        );
    }

    public Logger logTrace() {
        return logTrace(null);
    }

    public Long getLogInterval() {
        return logInterval;
    }

    public boolean isSuppressIfEqual() {
        return suppressIfEqual;
    }


}
