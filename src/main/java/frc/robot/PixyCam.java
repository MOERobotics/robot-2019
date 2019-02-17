package frc.robot;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2Line;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam implements Runnable{
    public volatile boolean isRunning = false;
    private SPILink pixySPI = new SPILink();
    private Pixy2 pixyCam = Pixy2.createInstance(pixySPI);
    private Thread pixyThread;

    Pixy2Line.Vector[] vec;

    public void init(){
        //Init pixycam
        pixyCam.init();
    }
    public void start(){
        pixyThread = new Thread(this);
        this.isRunning = true;
        pixyThread.start();
    }
    public void stop(){
        if(this.isRunning){
            this.isRunning = false;
            pixyThread.interrupt();
        }
    }
    public void run(){
        while(this.isRunning){
            //get vectors
            try{
                pixyCam.getLine().getAllFeatures();
                Pixy2Line.Vector[] tmp = pixyCam.getLine().getVectors();
                synchronized (this) {
                    this.vec = tmp;
                }
            }catch(Exception e){
                e.printStackTrace();
            }



        }
    }
    public Pixy2Line.Vector[] getLastVector(){
        synchronized (this) {
            return vec;
        }
    }
}
