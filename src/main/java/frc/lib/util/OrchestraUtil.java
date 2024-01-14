package frc.lib.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

public class OrchestraUtil {
    
    private static Orchestra orchestra = new Orchestra();

    public static void add(TalonFX...talonFXs){
        for(TalonFX fx : talonFXs){
            orchestra.addInstrument(fx);
        }
    }

    public static void play(String fileName){
        orchestra.loadMusic(fileName + ".chrp");
        orchestra.play();
    }

    public static boolean isPlaying(){
        return orchestra.isPlaying();
    }

    public static void stop(){
        orchestra.stop();
    }
}
