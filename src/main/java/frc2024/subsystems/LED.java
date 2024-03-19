// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.ScreamUtil;
import com.team4522.lib.util.COTSFalconSwerveConstants.driveGearRatios;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLEDBuffer.IndexedColorIterator;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants.LEDConstants;
import frc2024.Constants.Ports;

/** Add your docs here. */
public class LED extends SubsystemBase{
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private final int length = LEDConstants.STRIP_LENGTH;

    public enum LEDState{
        SOLID, STROBE, BREATHE, RAINBOW, WAVE, STRIPES, LARSON, SCALED_TARGET;
    }

    public LED(){
        leds = new AddressableLED(Ports.LED_ID);
        buffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);
        leds.setLength(length);
        leds.setData(buffer);
        leds.start();
    }

    // From FRC 6328 Mechanical Advantage
    // https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/leds/Leds.java

    private void solid(Color color) {
        if (color != null) {
            for (int i = 0; i < length; i++) {
                buffer.setLED(i, color);
            }
        }
    }

    private void solid(double percent, Color color) {
        for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
            buffer.setLED(i, color);
        }
    }

    private void strobe(Color color, double duration) {
        boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        solid(on ? color : Color.kBlack);
    }

    private void breathe(Color c1, Color c2, double duration) {
        breathe(c1, c2, duration, Timer.getFPGATimestamp());
    }

    private void breathe(Color c1, Color c2, double duration, double timestamp) {
        double x = ((timestamp % LEDConstants.BREATHE_DURATION) / LEDConstants.BREATHE_DURATION) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        solid(new Color(red, green, blue));
    }

    private void rainbow(double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
        double xDiffPerLed = 180.0 / cycleLength;
        for (int i = 0; i < length; i++) {
          x += xDiffPerLed;
          x %= 180.0;
          if (i >= 0) {
            buffer.setHSV(i, (int) x, 255, 255);
          }
        }
    }

    private void wave(Color c1, Color c2, double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = 0; i < length; i++) {
          x += xDiffPerLed;
          if (i >= 0) {
            double ratio = (Math.pow(Math.sin(x), LEDConstants.BREATHE_DURATION) + 1.0) / 2.0;
            if (Double.isNaN(ratio)) {
              ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.BREATHE_DURATION) + 1.0) / 2.0;
            }
            if (Double.isNaN(ratio)) {
              ratio = 0.5;
            }
            double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
            double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
            double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
            buffer.setLED(i, new Color(red, green, blue));
          }
        }
    }

    private void stripes(List<Color> colors, int length, double duration) {
        int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
        for (int i = 0; i < length; i++) {
          int colorIndex =
              (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
          colorIndex = colors.size() - 1 - colorIndex;
          buffer.setLED(i, colors.get(colorIndex));
        }
    }

    public void scaledTarget(Color color, double currentValue, double targetValue){
        if(targetValue == 0) return;
        int mapped = (int) Math.round(Conversions.mapRange(currentValue, 0, targetValue, 0, length));
        for(int i = 0; i < mapped; i++){
            if(i >= length) return;
            buffer.setLED(i, color);
        }
        for(int i = mapped; i < length; i++){
            if(i >= length) return;
            buffer.setLED(i, Color.kBlack);
        }
    }

    @Override
    public void periodic() {
        leds.setData(buffer);
        // System.out.println("test");
    }

    public Command solidCommand(Color color){
        return run(() -> solid(color)).ignoringDisable(true);
    }

    public Command solidCommand(Color color, double percent){
        return run(() -> solid(percent, color)).ignoringDisable(true);
    }

    public Command strobeCommand(Color color, double duration){
        return run(() -> strobe(color, duration)).ignoringDisable(true);
    }

    public Command strobeCommand(Supplier<Color> color, double duration){
        return run(() -> strobe(color.get(), duration)).ignoringDisable(true);
    }

    public Command breatheCommand(Color color1, Color color2, double duration){
        return run(() -> breathe(color1, color2, duration)).ignoringDisable(true);
    }

    public Command rainbowCommand(double cycleLength, double duration){
        return run(() -> rainbow(cycleLength, duration)).ignoringDisable(true);
    }

    public Command waveCommand(Color color1, Color color2, double cycleLength, double duration){
        return run(() -> wave(color1, color2, cycleLength, duration)).ignoringDisable(true);
    }

    public Command waveCommand(Supplier<Color> color1, Supplier<Color> color2, double cycleLength, double duration){
        return run(() -> wave(color1.get(), color2.get(), cycleLength, duration)).ignoringDisable(true);
    }

    public Command stripeCommand(List<Color> colors, int length, double duration){
        return run(() -> stripes(colors, length, duration)).ignoringDisable(true);
    }

    public Command scaledTargetCommand(Color color, DoubleSupplier currentValue, DoubleSupplier targetValue){
        return run(() -> scaledTarget(color, currentValue.getAsDouble(), targetValue.getAsDouble())).ignoringDisable(true);
    }
}
