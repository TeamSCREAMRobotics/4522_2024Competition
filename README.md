# BaseSwerve </br>

<br>This template includes everything we require for a basic Swerve Drive robot.
<br>
<br><b>Includes:</b>
   * Phoenix 6 Implementation
   * Basic Swerve Code
   * PathPlanner functionality
   * Basic examples for autonomous routines
   * Shuffleboard functionality
   * Various Utilty classes


<br><br>**CHANGE TEAM NUMBER**
----
Open the Command Palette (Ctrl+Shift+P) then type ```Set Team Number```.


<br><br>**Setting Constants**
----
The following things must be adjusted to your robot and module's specific constants in the [```Constants.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) file (all distance units must be in meters, and rotation units in radians)</br>
1. Gyro Settings: [```pigeonID```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) and [```invertGyro```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) (ensure that the gyro rotation is CCW+ (Counter Clockwise Positive)
2. [```chosenModule```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): 
<br>Set the module and drive ratio you are using here.
<br>For our uses, it will typically be the SDS MK4/MK4i Module. Make sure to select the correct gear ratio - for us it will most likely be Level 3.
<br>This will automatically set these constants required for the module to function properly:
    * Wheel Circumference
    * Angle Motor Invert
    * Drive Motor Invert
    * CANcoder Sensor Invert
    * Angle Motor Gear Ratio
    * Drive Motor Gear Ratio
    * Angle Falcon Motor PID Values
    
4. [```trackWidth```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Center to Center distance of left and right modules in meters.
5. [```wheelBase```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Center to Center distance of front and rear module wheels in meters.
6. [```wheelCircumference```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Cirumference of the wheel (including tread) in meters. <br><b>This value will be automatically set by the selected module.</b>
7. [```driveGearRatio```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Total gear ratio for the drive motor. <br><b>This value will be automatically set by the selected module.</b>
8. [```angleGearRatio```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Total gear ratio for the angle motor. <br><b>This value will be automatically set by the selected module.</b>
9. [```CANcoderInvert```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) and [```angleMotorInvert```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Both must be set such that they are CCW+. <br><b>This value will be automatically set by the selected module, but checking is recommended.</b>
10. [```driveMotorInvert```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): This can always remain false, since you set your offsets in step #11 such that a positive input to the drive motor will cause the robot to drive forwards. <br><b>This value will be automatically set by the selected module.</b>

11. [```Module Specific Constants```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): set the Can Id's of the motors and CANcoders for the respective modules, see the next step for setting offsets.
12. Setting Offsets
    * For finding the offsets you need to line up all modules straight. Use a piece of wood/metal to do this.
    * Point the bevel gears of all the wheels in the same direction (either facing left or right), where a postive input to the drive motor drives the robot forward (Use phoenix tuner to test this). If for some reason you set the offsets with the wheels backwards, you can change the [```driveMotorInvert```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) value to fix.
    * Open Shuffleboard and go to the smartdashboard tab. You will see 4 printouts called "Mod 0 CANcoder", "Mod 1 CANcoder", etc. 
    <br>If you have already straightened the modules, copy those 4 numbers to their respective [```angleOffset```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) parameter in the SwerveModuleConstants for each module.
    <br><b>Note: The CANcoder values printed to Shuffleboard are in degrees, when copying the values to [```angleOffset```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) you must use ```Rotation2d.fromDegrees(value_here)```.</b>

13. Angle Motor PID Values: <br><b>This value will be automatically set through the selected module. If you prefer it to be more or less aggressive, see instructions below</b> 
    * To tune start with a low P value (0.01).
    * Multiply by 10 until the module starts oscilating around the set point
    * Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module doesn't oscillate around the setpoint.
    * If there is any overshoot you can add in some D by repeating the same process, leave at 0 if not. Always leave I at 0.

14. [```maxSpeed```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): In Meters Per Second. [```maxAngularVelocity```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): In Radians Per Second. For these you can use the theoretical values, but it is better to physically drive the robot and find the actual max values.

15. [```driveKS```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java), [```driveKV```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java), and [```driveKA```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java)
<br>Leave these as the default values. If for some reason they require a change, you can use the WPILib characterization tool, found [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html). You will need to lock the modules straight forward, and complete the characterization as if it was a standard tank drive.
17. [```driveKP```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): 
<br>After inserting the KS, KV, and KA values into the code, tune the drive motor kP until it doesn't overshoot or oscillate around a target velocity.
<br>Leave [```driveKI```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java), [```driveKD```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java), and [```driveKF```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) at 0.0.


<br><br>**Controller Mappings**
----
The code is natively setup to use a Xbox controller, though other controllers will work. </br>
<br><b>Note: To add additional button bindings, create methods in [```Controlboard.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/controlboard/Controlboard.java) and reference them in [```RobotContainer.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/RobotContainer.java).
See [```configButtonBindings```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/RobotContainer.java).</b>
* Left Stick: Translation Control (forwards and sideways movement)
* Right Stick: Rotation Control </br>
* Back Button: Zero Gyro (useful if the gyro drifts mid match, just rotate the robot forwards, and press Back to rezero)
* Start Button: Toggles field-centric mode



<br><br>**PathPlanner/Autonomous Configuration**
----
<br>The following relates to using PathPlanner.
<br>Refer to the [PathPlanner wiki](https://github.com/mjansen4857/pathplanner/wiki) for additional help.

<br><b>Using the FRC PathPlanner application</b>
* Open the FRC PathPlanner application (or install it if it is not already) and switch to the current project.
   * Check that there is a path named "ExamplePath" - if there is not, use Add Path to create it.
   * Use Add Path to create any additional paths.
   * Use event markers to create events along a path. 

<br><b>Controllers</b>
* [```pathTranslationController```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): The PID controller used for following and correcting translation for the path's trajectory.
* [```pathRotationController```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): The PID controller used for following and correcting rotation for the path's trajectory.
* Use the guide in Angle Motor PID Values to tune these controllers.

<br><b>Using paths to create Autonomous routines</b>
* Create methods that return individual trajectories with their respective constraints. See [```AutoTrajectories.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/auto/AutoTrajectories.java).
* Add events with their corresponding command. See [```AutoEvents.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/auto/AutoEvents.java) and [```RobotContainer.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/RobotContainer.java)
* Create autonomous routines using Sequential Commands, creating a list of actions. See [```AutoRoutines.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/auto/AutoRoutines.java).
* Add auto routines to the auto chooser in [```RobotContainer.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/RobotContainer.java).



<br><br>**Shuffleboard Configuration**
----
<br>The following relates to using tabs and entries with Shuffleboard.
<br>Refer to the [Shuffleboard wiki](https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/index.html) for additional help.

<br><b>Creating tabs and entries</b>
* Create a class extending ShuffleboardTabBase and structure it as follows:
   * GenericEntry objects for each of the values you want to display/get with Shuffleboard.
   * Initialize the tab and use create methods from [```ShuffleboardTabBase.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabBase.java) to create entries with the respective data types in [```createEntries()```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabBase.java)
   * Either set or get the entry's value in [```periodic()```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabManager.java)
   * See [```SwerveTab.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/tabs/SwerveTab.java) for an example of this structure.
 
<br><b>Putting tabs on Shuffleboard</b>
* Add tabs to the list in [```ShuffleboardTabManager.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabManager.java), either as a debug tab or regular tab.
   * Debug tabs will only be shown if [```debug```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabManager.java) is true
      * Displaying to Shuffleboard is resource-intensive, so make sure it is true only when you are debugging.

<br><b>Using the Shuffleboard application</b>
* Shuffleboard should be automatically installed, but you may have to select it in DriverStation.
* All tabs and corresponding entries should appear with the values/names they are set with, assuming you have deployed the code.
   * If they do not appear, simply restart the robot code and re-open Shuffleboard.
