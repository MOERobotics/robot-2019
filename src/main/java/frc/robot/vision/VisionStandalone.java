package frc.robot.vision;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class VisionStandalone {
    private static DatagramSocket socket;
    private static byte[] buf = new byte[256];

    public static void main(String... argv) throws Exception {

        socket = new DatagramSocket(5801);
        run();
//        VisionListener listener = new VisionListener();
//        listener.start();
//
//        Scanner scan = new Scanner(System.in);
//        scan.next();
//
//        listener.die = true;

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

            System.out.println(received);

            if (received.equals("end")) {
                running = false;
                continue;
            }
            socket.send(packet);
        }
        socket.close();
    }
}
