package frc.robot.vision;

import java.net.ServerSocket;
import java.net.Socket;

public class VisionListener extends Thread{

    VisionData currentData;
    boolean die = false;

    @Override
    public void run(){
        try{
            ServerSocket socketServer = new ServerSocket(5801);

            while(!die){
                Socket sock = socketServer.accept();
                VisionClient client = new VisionClient(this,sock);
                client.start();
            }
            

        } catch(Exception ignored) {

        }
    }

}
