// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            RobotContainer.Winch.goToPosition(0, 2),
            RobotContainer.Wrist.goToPosition(0, 1)
            )
        );
    }
}
