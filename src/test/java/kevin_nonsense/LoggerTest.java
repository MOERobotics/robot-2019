package kevin_nonsense;

import org.junit.*;

import java.io.ByteArrayOutputStream;
import java.io.StringWriter;
import static org.junit.Assert.*;

public class LoggerTest {

    ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
    Logger testLogger;

    @Before
    public void setUp() throws Exception {
        testLogger = new Logger("test",outputStream);

    }

    @Test
    public void logMessage() {
        String classname = new Object(){}.getClass().getEnclosingMethod().getName();
        outputStream.reset();
        testLogger
            .alertIfEqual()
            .unsetLogInterval()
            .logMessage(classname);
        String output = outputStream.toString();
        assertTrue(output.contains(classname));
        assertTrue(output.contains(testLogger.key));
    }


    @Test
    public void logTrace() {
        String classname = new Object(){}.getClass().getEnclosingMethod().getName();
        outputStream.reset();
        testLogger
            .alertIfEqual()
            .unsetLogInterval()
            .logTrace();
        String output = outputStream.toString();
        assertTrue(output.contains(classname));
        assertTrue(output.contains(testLogger.key));
    }

    @Test
    public void logValue() {
        Integer value = 5;
        outputStream.reset();
        testLogger
            .alertIfEqual()
            .unsetLogInterval()
            .logValue(value);
        String output = outputStream.toString();
        assertTrue(output.contains(value.toString()));
        assertTrue(output.contains(testLogger.key));
    }

    @Test
    public void logValueTimer() {
        Integer value = 5;
        logValue();
        outputStream.reset();
        testLogger
            .alertIfEqual()
            .logOnce()
            .logValue(value);
        String output = outputStream.toString();
        assertEquals(output,"");
        logValue();
        outputStream.reset();
        testLogger
            .alertIfEqual()
            .minLogInterval(0L)
            .logValue(value);
        output = outputStream.toString();
        assertTrue(output.contains(value.toString()));
        assertTrue(output.contains(testLogger.key));
        logValue();
    }

    @Test
    public void logValueChanged() {
        logValue();
        outputStream.reset();
        testLogger
            .suppressIfEqual()
            .unsetLogInterval()
            .logValue(5);
        String output = outputStream.toString();
        assertEquals(output,"");
        outputStream.reset();
        testLogger
            .suppressIfEqual()
            .unsetLogInterval()
            .logValue(6);
        output = outputStream.toString();
        assertTrue(output.contains(Integer.valueOf(6).toString()));
        assertTrue(output.contains(testLogger.key));
        outputStream.reset();
        testLogger
            .suppressIfEqual()
            .unsetLogInterval()
            .logValue(6);
        output = outputStream.toString();
        assertEquals(output,"");
    }

    @Test
    public void ignoreValueIfNull() {
        String classname = new Object(){}.getClass().getEnclosingMethod().getName();
        outputStream.reset();
        testLogger
                .suppressIfEqual()
                .unsetLogInterval()
                .logMessage(classname);
        String output = outputStream.toString();
        assertTrue(output.contains(classname));
        assertTrue(output.contains(testLogger.key));
        outputStream.reset();
        testLogger.logMessage(classname);
        output = outputStream.toString();
        assertTrue(output.contains(classname));
        assertTrue(output.contains(testLogger.key));
    }

}
