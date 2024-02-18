package com.team1706;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc2024.Constants;
import frc2024.subsystems.swerve.Swerve;

public class SmartShooting {

    //1706 mock-up, shoot while moving from Rapid React 2022 Season
  public static Translation2d calculateVirtualTarget(Swerve swerve, Translation2d target){
    /* Creates the field relative speed variables */

    /* Sets the field relative speed variables */
    ChassisSpeeds m_fieldRelVel = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(), swerve.getRotation());
    ChassisSpeeds m_lastFieldRelVel = m_fieldRelVel;
    FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, Constants.LOOP_TIME_SEC);

    /* Finds the robot distance to the target */
    Translation2d robotToGoal = target.minus(swerve.getPose().getTranslation());
    double dist = robotToGoal.getDistance(new Translation2d());
    
    /* Gets the time it takes for the note to leave the shooter TODO necessary to have it a changing value? (refrence constants) */
    double shotTime = 0.3;//ShooterConstants.timeToGoalMap.get(dist);
    Translation2d movingGoalLocation = new Translation2d();
    for(int i=0; i<5; i++){
      /* Calculates the  distance and time for the virtual target */
        double virtualGoalX = target.getX() - shotTime * (m_fieldRelVel.vxMetersPerSecond + m_fieldRelAccel.ax * 0.01 /* ShooterConstants.kAccelCompFactor */);
        //TODO shooter constant comp factor? It is necessary but why? How do you get the value?
        double virtualGoalY = target.getY() - shotTime * (m_fieldRelVel.vyMetersPerSecond + m_fieldRelAccel.ay * 0.01 /* ShooterConstants.kAccelCompFactor */);
        //TODO shooter constant comp factor? It is necessary but why? How do you get the value?
        Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
        Translation2d toGoalDistance = testGoalLocation.minus(swerve.getPose().getTranslation());
        double newShotTime = 0.3;//ShooterConstants.timeToGoalMap.get(toGoalDistance.getDistance(new Translation2d()));
        
        //Used to smooth the angle adjustment of the robot
        if(Math.abs(newShotTime-shotTime) <= 0.01){
            i = 4;
        }
        if(i == 4){
            movingGoalLocation = testGoalLocation;
        }
        else shotTime = newShotTime;
    }

    return movingGoalLocation;
  }
}
