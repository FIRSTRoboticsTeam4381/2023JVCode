package frc.robot.autos;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.VisionLineup;

public final class Autos {

    /**
     * Events to be used in all autos built with pathplanner
     */
    public static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        //Turns on Limelight Leds, good indicator of step working
        Map.entry("lime", new InstantCommand(() -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3))),
        //Stops robot from drifting when stopped at stop point
        Map.entry("stop", new InstantCommand(() -> RobotContainer.s_Swerve.drive(new Translation2d(0,0), 0, false, true))),

        Map.entry("placeTop", new SequentialCommandGroup(
            SpecialistPositions.topPlacement(),
            RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(0.5)
            //RobotContainer.Winch.goToPosition(10000, 200) // ERROR is the RANGE -- THis is for John because he cant remember what it is.
        )),
        Map.entry("placeMid", new SequentialCommandGroup(
            SpecialistPositions.midPlacement(),
            RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(0.5)
            //RobotContainer.Winch.goToPosition(10000, 200) // ERROR is the RANGE -- THis is for John because he cant remember what it is.
        )),
        Map.entry("zero", SpecialistPositions.zero()),
        Map.entry("balance", RobotContainer.balanceRobot),
        Map.entry("lowerArm", RobotContainer.Winch.goToPosition(10010, 200)),
        Map.entry("grabCONE", new SequentialCommandGroup(
            SpecialistPositions.offGround(),
            RobotContainer.Gripper.coneGripper(),
            new WaitCommand(0.75),
            SpecialistPositions.zero())),
        Map.entry("readyGrab", SpecialistPositions.offGround()),
        Map.entry("grabCube", new SequentialCommandGroup(
            RobotContainer.Winch.goToPosition(33000, 200),
            RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(1),
            RobotContainer.Winch.goToPosition(25000, 200)
        )),
         Map.entry("LEDFlash", new SequentialCommandGroup(
            RobotContainer.leds.setColorsCommand(1, 0, 0),
            new WaitCommand(0.5),
            RobotContainer.leds.setColorsCommand(0, 0, 0),
            new WaitCommand(0.5),
            RobotContainer.leds.setColorsCommand(1, 0, 0),
            new WaitCommand(0.5),
            RobotContainer.leds.setColorsCommand(0, 0, 0),
            new WaitCommand(0.5),
            RobotContainer.leds.setColorsCommand(1, 0, 0),
            new WaitCommand(0.5),
            RobotContainer.leds.setColorsCommand(0, 0, 0),
            new WaitCommand(0.5),
            RobotContainer.leds.setColorsCommand(0, 1, 0),
            new WaitCommand(0.5)
        )),
        Map.entry("cubeLineup", new VisionLineup(RobotContainer.s_Swerve, RobotContainer.lime, RobotContainer.leds, 1, true)),
        Map.entry("aprilLineup", new VisionLineup(RobotContainer.s_Swerve, RobotContainer.lime, RobotContainer.leds, 0, false)),
        Map.entry("moveAndMid", new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> RobotContainer.s_Swerve.drive(new Translation2d(1,0), 0, false, true)),
                new WaitCommand(0.5),
                SpecialistPositions.midPlacement()
            ),
            new InstantCommand(() -> RobotContainer.s_Swerve.drive(new Translation2d(0,0), 0, false, true)),
            RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(0.5)
        )),
        Map.entry("tipDown", new InstantCommand(() -> RobotContainer.Wrist.goToPosition(0.28, 0.003)))
        
    ));

    /**
     * Swerve auto builder, use to runs path routines in autonomous
     */
    private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        RobotContainer.s_Swerve::getPose, // Pose2d supplier
        RobotContainer.s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        RobotContainer.s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        RobotContainer.s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );

    /**
     * Example PathPlanner Auto
     * @return Autonomous command
     */
    public static Command exampleAuto(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("PathPlannerTest", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command TopPlacementAuto(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("TopPlacementAuto", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command TopPlacementBalance(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("TopPlacementBalance", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command PlaceCubeandCone(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("PlaceCube&Cone", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command BackCubeReverse(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("BackCubeReverse", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command BackConeReverse(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("BackConeReverse", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command FrontConeReverse(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("FrontConeReverse", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command FrontCubeReverse(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("FrontCubeReverse", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command BackCubePickup(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("BackCubePickup", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command BackCubePickupBlue(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("BackCubePickupBlue", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command CubeOverBalance(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("CubeOverBalance", 
            new PathConstraints(1.5, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command BackConeCubePickup(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("BackConeCubePickup", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command BackLinePickup(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("BackLinePickup", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    public static Command BackVisionPickup(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("BackVisionPickup", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }
    /**
     * Blank Autonomous to be used as default dashboard option
     * @return Autonomous command
     */
    public static Command none(){
        return Commands.none();
    }

}
