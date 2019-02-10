package frc.robot.vision;

import java.util.Scanner;

public class VisionStandalone {
    public static void main(String... argv) {
        VisionListener listener = new VisionListener();
        listener.start();

        Scanner scan = new Scanner(System.in);
        scan.next();

        listener.die = true;

    }
}
