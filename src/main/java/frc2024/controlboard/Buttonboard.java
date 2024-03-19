package frc2024.controlboard;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class Buttonboard {

    public static class ButtonboardPort{
        public int joystickPort;
        public int id;
        public boolean invert;
        public ButtonboardPort(int joystickPort, int id, boolean invert){
            this.joystickPort = joystickPort;
            this.id = id;
            this.invert = invert;
        }
        public ButtonboardPort(int joystickPort, int id){
            this(joystickPort, id, false);
        }
    }

    public static enum Direction{
        UP, UP_LEFT, LEFT, DOWN_LEFT, DOWN, DOWN_RIGHT, RIGHT, UP_RIGHT, CENTER;
    }

    private Joystick[] usbDevices = new Joystick[2];
    HashMap<Integer, ButtonboardPort> buttonMap = new HashMap<Integer, ButtonboardPort>();
    HashMap<Integer, ButtonboardPort> switchMap = new HashMap<Integer, ButtonboardPort>();
    private final int joystickXAxis;
    private final int joystickYAxis;
    private final int joystickIndex = 1;//the driverstation port that the joystick is on.

    // HashMap<Direction, ButtonboardPort> bigSwitchMap = new HashMap<Direction, ButtonboardPort>();


    public Buttonboard(int portA, int portB) {
        usbDevices[0] = new Joystick(portA);
        usbDevices[1] = new Joystick(portB);

        /* Buttons on the button board going left to right */
        buttonMap.put(1, new ButtonboardPort(0, 1));
        buttonMap.put(2, new ButtonboardPort(0, 2));
        buttonMap.put(3, new ButtonboardPort(0, 3));
        buttonMap.put(4, new ButtonboardPort(0, 4));
        buttonMap.put(5, new ButtonboardPort(0, 5));
        buttonMap.put(6, new ButtonboardPort(0, 6));
        buttonMap.put(7, new ButtonboardPort(0, 7));
        buttonMap.put(8, new ButtonboardPort(0, 8));
        buttonMap.put(9, new ButtonboardPort(0, 9));
        buttonMap.put(10, new ButtonboardPort(0, 10));
        buttonMap.put(11, new ButtonboardPort(0, 11));
        buttonMap.put(12, new ButtonboardPort(0, 12));

        /* Switches on the button board going left to right */
        switchMap.put(1, new ButtonboardPort(1, 4));
        switchMap.put(2, new ButtonboardPort(1, 1));
        switchMap.put(3, new ButtonboardPort(1, 2));
        switchMap.put(4, new ButtonboardPort(1, 3));
        switchMap.put(5, new ButtonboardPort(1, 5));

        joystickXAxis = 0;
        joystickYAxis = 1;
    }

    public boolean getRawButton(int button){
        ButtonboardPort selectedButton = buttonMap.get(button);
        
        if(selectedButton != null){
            if(selectedButton.invert) return !usbDevices[selectedButton.joystickPort].getRawButton(selectedButton.id);
            else return usbDevices[selectedButton.joystickPort].getRawButton(selectedButton.id);
        }
        DriverStation.reportError("error in buttonboard getRawButton: " + button, false);
        return false;

    }

    public boolean getRawButtonPressed(int button){
        ButtonboardPort selectedButton = buttonMap.get(button);
        
        if(selectedButton != null){
            if(selectedButton.invert) return usbDevices[selectedButton.joystickPort].getRawButtonReleased(selectedButton.id);
            else return usbDevices[selectedButton.joystickPort].getRawButtonPressed(selectedButton.id);
        }
        DriverStation.reportError("error in buttonboard getRawButtonPressed: " + button, false);
        return false;
    }

    public boolean getRawButtonReleased(int button){
        ButtonboardPort selectedButton = buttonMap.get(button);
        
        if(selectedButton != null){
            if(selectedButton.invert) return usbDevices[selectedButton.joystickPort].getRawButtonPressed(selectedButton.id);
            else return usbDevices[selectedButton.joystickPort].getRawButtonReleased(selectedButton.id);
        }
        DriverStation.reportError("error in buttonboard getRawButtonReleased: " + button, false);
        return false;
    }

    public boolean getRawSwitch(int switchID){
        ButtonboardPort selectedSwitch = switchMap.get(switchID);

        if(selectedSwitch != null){
            if(selectedSwitch.invert) return !usbDevices[selectedSwitch.joystickPort].getRawButton(selectedSwitch.id);
            else return usbDevices[selectedSwitch.joystickPort].getRawButton(selectedSwitch.id);
        }
        DriverStation.reportError("error in buttonboard getRawSwitch: " + switchID, false);
        return false;
    }

    public double getBigSwitchX(){
        return usbDevices[joystickIndex].getRawAxis(joystickXAxis);
    }

    public double getBigSwitchY(){
        return usbDevices[joystickIndex].getRawAxis(joystickYAxis);
    }

    public Direction getBigSwitchDirection(){
        int x = (int)usbDevices[joystickIndex].getRawAxis(joystickXAxis);
        int y = (int)usbDevices[joystickIndex].getRawAxis(joystickYAxis);

        if(x == 1 && y == 0){
            return Direction.RIGHT;
        } else if(x == 1 && y == 1){
            return Direction.UP_RIGHT;
        } else if(x == 0 && y == 1){
            return Direction.UP;
        } else if(x == -1 && y == 1){
            return Direction.UP_LEFT;
        } else if(x == -1 && y == 0){
            return Direction.LEFT;
        } else if(x == -1 && y == -1){
            return Direction.DOWN_LEFT;
        } else if(x == 0 && y == -1){
            return Direction.DOWN;
        } else if(x == 1 && y == -1){
            return Direction.DOWN_RIGHT;
        } else{
            return Direction.CENTER;
        }
    }
}