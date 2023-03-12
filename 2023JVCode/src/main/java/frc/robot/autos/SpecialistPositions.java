// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class SpecialistPositions {
    public static Command topPlacement(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(new WaitCommand(.2), RobotContainer.Extender.goToPosition(49.9, .2)),
            new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(49.9, .2),
            RobotContainer.Winch.goToPosition(90000, 500),
            RobotContainer.Wrist.goToPosition(0.324, 0.003)
        ));
    }
    public static Command zero(){
        
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(new WaitCommand(.5), RobotContainer.Winch.goToPosition(0, 1)),
            new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(0, 1), 
            RobotContainer.Winch.goToPosition(0, 500),
            RobotContainer.Wrist.goToPosition(0.05, 0.04)
            )
        );
    }
    /*public static Command autoGrabCone(){
        return new SequentialCommandGroup(
            
            RobotContainer.Wrist.goToPosition(0, 10),
            RobotContainer.Winch.goToPosition(0, 10),    
            RobotContainer.Extender.goToPosition(42, .2),
            RobotContainer.Gripper.coneGripper(),
            new WaitCommand(.5),
            new ParallelCommandGroup(
                //RobotContainer.Wrist.goToPosition(9.5, .2),
                RobotContainer.Winch.goToPosition(34707, 500)//26300
            ),
            RobotContainer.Extender.goToPosition(22, .2),//19.2
            RobotContainer.Gripper.coneGripper(),
            new WaitCommand(.5),
            RobotContainer.Winch.goToPosition(0, 1),
            new ParallelCommandGroup(
                RobotContainer.Wrist.goToPosition(0, 1),
                RobotContainer.Extender.goToPosition(25, 1)
            )
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
    }*/

    public static Command autoGrabCone(){
        return new SequentialCommandGroup(
            
            RobotContainer.Wrist.goToPosition(0, 10),
            RobotContainer.Winch.goToPosition(0, 10),    
            RobotContainer.Extender.goToPosition(42, .2),
            new WaitCommand(.5),
            new ParallelCommandGroup(
                //RobotContainer.Wrist.goToPosition(9.5, .2),
                RobotContainer.Winch.goToPosition(26300, 500),
                RobotContainer.Extender.goToPosition(20, .2)
            ),
            RobotContainer.Gripper.coneGripper(),
            new WaitCommand(.3),
            RobotContainer.Winch.goToPosition(0, 500),
            new ParallelCommandGroup(
                RobotContainer.Wrist.goToPosition(0, 1),
                RobotContainer.Extender.goToPosition(25, 1)
            )
        );
    }
    public static Command autoGrabCube(){
        return new SequentialCommandGroup(
            RobotContainer.Extender.goToPosition(39, .2),
            RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(.5),
            RobotContainer.Winch.goToPosition(25000, 300),
            RobotContainer.Extender.goToPosition(21, .2),
            RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(1.5),
            RobotContainer.Winch.goToPosition(0, 1)
            
        );
    }
    public static Command midPlacement(){
        return new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(38.166, 0.2),
            RobotContainer.Winch.goToPosition(66664, 500),
            RobotContainer.Wrist.goToPosition(0.1695, .003)
        );
    }
    public static Command offGround(){
        return new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(0, 1),
            RobotContainer.Winch.goToPosition(149683, 500),
            RobotContainer.Wrist.goToPosition(0.1842, 0.003)
        );
    }

}
