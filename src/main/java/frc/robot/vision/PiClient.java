package frc.robot.vision;

public class PiClient {
    private Thread piVisionThread;
    private static PiClient piClientInstance;
    private PiVisionServiceThread piVisionServiceThread;

    private PiClient() {
        piVisionServiceThread = new PiVisionServiceThread();
        piVisionThread = new Thread(piVisionServiceThread);
        piVisionThread.start();
    }

    public static PiClient getInstance() {
        if (piClientInstance == null) {
//            System.out.println("Creating new instance");
            piClientInstance = new PiClient();
        }
        return piClientInstance;
    }

    public void stop() {
        piVisionThread.interrupt();
    }

    public int[] getCentroidXY() {
//        System.out.println("Creating centroid x y from pi client thread");
        return piVisionServiceThread.getCentroidXY();
    }
}
