package frc.robot.vision;

import java.io.InputStream;
import java.net.Socket;

public class VisionClient extends Thread{
    VisionListener parent;
    Socket sock;

    boolean die = false;

    public VisionClient(VisionListener parent, Socket socket){
        this.parent = parent;
        this.sock = socket;
    }

    @Override
    public void run(){
        try {
            InputStream inStream = sock.getInputStream();
            VisionData data = new VisionData();
            String input = "";
            while (!die) {
                int character = inStream.read();
                switch (character){
                    case '\n':
                    case '\0':
                        //End of Message


                        System.out.println(input);

                        parent.currentData = data;

                        data = new VisionData();
                        input = "";
                        break;
                    default:
                        //Add to message
                        input += (char)character;
                        break;
                }
            }
        }catch(Exception ignored){

        }
    }
}
