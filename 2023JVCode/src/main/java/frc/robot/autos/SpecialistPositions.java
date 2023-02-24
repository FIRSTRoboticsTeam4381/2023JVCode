// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class SpecialistPositions {
    public static Command topPlacement(){
        return new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(49.9, .2),
            RobotContainer.Winch.goToPosition(145, 2),
            RobotContainer.Wrist.goToPosition(39.3, .2)
        );
    }
    public static Command zero(){
        
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(new WaitCommand(.5), RobotContainer.Winch.goToPosition(0, 1)),
            new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(0, 1),
            RobotContainer.Winch.goToPosition(0, 5),
            RobotContainer.Wrist.goToPosition(0, 1)
            )
        );
    }
    public static Command autoGrabCone(){
        return new SequentialCommandGroup(
            RobotContainer.Wrist.goToPosition(0, .2),
            RobotContainer.Winch.goToPosition(0, .2),    
            RobotContainer.Extender.goToPosition(42, .2),
            RobotContainer.Gripper.coneGripper(),
            new WaitCommand(.5),
            /*new ParallelCommandGroup(
                RobotContainer.Wrist.goToPosition(9.5, .2),
                RobotContainer.Winch.goToPosition(17.4, .2)
            ),*/
            RobotContainer.Extender.goToPosition(14, .2),
            RobotContainer.Gripper.coneGripper(),
            new WaitCommand(.5),
            RobotContainer.Winch.goToPosition(0, 1),
            RobotContainer.Wrist.goToPosition(0, 1),
            RobotContainer.Extender.goToPosition(25, 1)
        );
    }
    public static Command autoGrabCube(){
        return new SequentialCommandGroup(
            RobotContainer.Extender.goToPosition(39, .2),
            RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(.5),
            RobotContainer.Winch.goToPosition(36, .2),
            RobotContainer.Extender.goToPosition(22, .2),
            RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(1.5),
            RobotContainer.Winch.goToPosition(0, 1)
            
        );
    }
    public static Command midPlacement(){
        return new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(43.7, .2),
            RobotContainer.Winch.goToPosition(128, .2),
            RobotContainer.Wrist.goToPosition(18, .2)
        );
    }
    public static Command offGround(){
        return new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(0, 1),
            RobotContainer.Winch.goToPosition(206.8, .2),
            RobotContainer.Wrist.goToPosition(17.5, .2)
        );
    }
}
