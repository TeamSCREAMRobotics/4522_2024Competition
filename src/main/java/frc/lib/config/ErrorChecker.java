package frc.lib.config;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class ErrorChecker {

    public static final double BOOT_ALLOWANCE_SECONDS = 3.0;
    public static final int TRIES_TO_GENERATE_WARNING = 5;
    
    /**
     * This method takes a list of StatusCodes and returns true if they are all OK. When we configure our devices, we wrap all our our calls to the devices in this method
     * to tell us if the device has configured correctly, or if there are errors.
     */
    public static boolean hasConfiguredWithoutErrors(StatusCode... statusCodes){
        boolean okay = true;
        for(StatusCode statusCode : statusCodes){
            okay = okay && StatusCode.OK == statusCode;
        }
        return okay;
    }
    
    /**
     * This method does the actual configuration for the device. It repeatedly calls config.configureSettings() until there is a successful configuration or until it times out.
     * If printInfo is true, it will print if the configuration succeeded and how many tries it took
     */
    public static void configureDevice(DeviceConfiguration config, String name, double bootAllowanceSeconds, boolean printInfo){
        boolean goodConfiguration = false;
        Timer timer = new Timer();
        timer.reset();
        timer.start();

        int tries = 0;
        while(!goodConfiguration){
            tries++;

            if(timer.get() > bootAllowanceSeconds){
                if(printInfo) DriverStation.reportError("failed configuration for " + name +  " initialization, took " + tries + " tries.", false);
                return;
            }
            goodConfiguration = config.configureSettings();
            if(!goodConfiguration)
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
        if(printInfo && tries > TRIES_TO_GENERATE_WARNING)  DriverStation.reportWarning("Possible issue with " + name + ". Configuration took " + tries + " tries", false);
        else if(printInfo) System.out.println( "   " + name + " | configuration took " + tries + " tries. ");
    }

    /**
     * This method does the actual configuration for the device. It repeatedly calls config.configureSettings() until there is a successful configuration or until it times out.
     * If printInfo is true, it will print if the configuration succeeded and how many tries it took
     */
    public static void configureDevice(DeviceConfiguration config, String name, boolean printInfo){
        configureDevice(config, name, BOOT_ALLOWANCE_SECONDS, printInfo);
    }

    /**
     * This interface is where all of our logic for configuring each device goes.  
     */
    public static interface DeviceConfiguration{
        /**
         * This method does all of the configuration logic for the device and returns true only if the configuration is good. 
         */
        public boolean configureSettings();
    }
}