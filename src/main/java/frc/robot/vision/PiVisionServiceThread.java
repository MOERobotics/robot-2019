package frc.robot.vision;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class PiVisionServiceThread implements Runnable {
    private AtomicBoolean running = new AtomicBoolean(false);
    private AtomicInteger currentX = new AtomicInteger();
    private AtomicInteger currentY = new AtomicInteger();

    private int interval = 0;

    private DatagramSocket socket;
    private byte[] buffer = new byte[16];

    public PiVisionServiceThread() {
        try {
            socket = new DatagramSocket(5801);
            currentX.set(-1);
            currentY.set(-1);
        } catch (SocketException e) {
            e.printStackTrace();
        }
    }

    public void interrupt() {
        running.set(false);
    }

    boolean isRunning() {
        return running.get();
    }

    public int[] getCentroidXY() {
//        System.out.println("Returning new int[]");
        return new int[] { currentX.get(), currentY.get() };
    }

    public void run() {
        running.set(true);
//        System.out.println("Set running");

        while (running.get()) {
            try {
                Thread.sleep(interval);
//                System.out.println("Getting centroid");
                this.getCentroid();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private void getCentroid() {
        DatagramPacket datagramPacket = new DatagramPacket(buffer, buffer.length);

        try {
            socket.receive(datagramPacket);
        } catch (IOException e) {
            e.printStackTrace();
        }

        InetAddress address = datagramPacket.getAddress();
        int port = datagramPacket.getPort();

        datagramPacket = new DatagramPacket(buffer, buffer.length, address, port);

        String incomingMessage = new String(datagramPacket.getData(), 0, datagramPacket.getLength());

        String[] xyArray = (incomingMessage.split("\n"))[0].split(",");

        currentX.set(Integer.parseInt(xyArray[0]));
        currentY.set(Integer.parseInt(xyArray[1]));
    }
}