package com.team4522.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import frc2024.Constants.ShootState;

public class ShootStateInterpolatingTreeMap{
    private InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap heightInterpolator = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap velocityInterpolator = new InterpolatingDoubleTreeMap();

    public void put(double distance, ShootState shootState){
        angleInterpolator.put(distance, shootState.pivotAngle().getDegrees());
        heightInterpolator.put(distance, shootState.elevatorHeightInches());
        velocityInterpolator.put(distance, shootState.velocityRPM());
    }

    public ShootState get(double distance){
        return new ShootState(Rotation2d.fromDegrees(angleInterpolator.get(distance)), heightInterpolator.get(distance), velocityInterpolator.get(distance));
    }


}