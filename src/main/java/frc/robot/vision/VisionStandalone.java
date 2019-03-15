package frc.robot.vision;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class VisionStandalone {
    private static DatagramSocket socket;
    //Try byte buffer
    private static byte[] buf = new byte[256];

    public static void main(String... argv) throws Exception {

        socket = new DatagramSocket(5801);
        run();

    }

    private static void run() throws Exception {
        boolean running = true;

        while (running) {
            DatagramPacket packet
                    = new DatagramPacket(buf, buf.length);
            socket.receive(packet);

            InetAddress address = packet.getAddress();
            int port = packet.getPort();
            packet = new DatagramPacket(buf, buf.length, address, port);
            String received
                    = new String(packet.getData(), 0, packet.getLength());

            //System.out.println(received);
            //recievedArr[0] is the x value, [1] is y

            String[] recievedArr = received.split(",");

            System.out.println("X: " + recievedArr[0]);
            System.out.println("Y: " + recievedArr[1]);

            if (received.equals("end")) {
                running = false;
                continue;
            }

            socket.send(packet);
        }
        socket.close();
    }
}
