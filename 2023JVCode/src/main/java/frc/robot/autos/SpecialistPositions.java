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
            RobotContainer.Winch.goToPosition(16630, 200),
            RobotContainer.Wrist.goToPosition(0.7, 0.006)
        ));
    }
    public static Command zero(){
        
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(new WaitCommand(.2), RobotContainer.Winch.goToPosition(0, 1)),
            new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(0, 1), 
            RobotContainer.Winch.goToPosition(0, 200),
            RobotContainer.Wrist.goToPosition(0.385, 0.04)
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
            new ParallelCommandGroup(
                RobotContainer.Extender.goToPosition(0, 0.2),
                RobotContainer.Winch.goToPosition(11250, 100),
                RobotContainer.Wrist.goToPosition(0.7, .003)
            ),
            RobotContainer.Gripper.coneGripper()
        );
    }

    public static Command autoGrabCube(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                RobotContainer.Extender.goToPosition(0, 0.2),
                RobotContainer.Winch.goToPosition(12400, 100),
                RobotContainer.Wrist.goToPosition(0.7, .003)
            ),
            RobotContainer.Gripper.cubeGripper()
        );
    }

    /*public static Command autoGrabCone(){
        return new SequentialCommandGroup(
            
            RobotContainer.Wrist.goToPosition(0, 10),
            RobotContainer.Winch.goToPosition(0, 200),    
            RobotContainer.Extender.goToPosition(42, .2),
            new WaitCommand(.5),
            new ParallelCommandGroup(
                //RobotContainer.Wrist.goToPosition(9.5, .2),
                RobotContainer.Winch.goToPosition(3757, 200),
                RobotContainer.Extender.goToPosition(20, .2)
            ),
            RobotContainer.Gripper.coneGripper(),
            new WaitCommand(.3),
            RobotContainer.Winch.goToPosition(0, 200),
            new ParallelCommandGroup(
                RobotContainer.Wrist.goToPosition(0, 1),
                RobotContainer.Extender.goToPosition(25, 1)
            )
        );
    }
    public static Command autoGrabCube(){
        return new SequentialCommandGroup(
            RobotContainer.Extender.goToPosition(39, .2),
            //RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(.5),
            RobotContainer.Winch.goToPosition(3571, 200),
            RobotContainer.Extender.goToPosition(21, .2),
            RobotContainer.Gripper.cubeGripper(),
            new WaitCommand(1.5),
            RobotContainer.Winch.goToPosition(0, 1)
            
        );
    }*/

    public static Command midPlacement(){
        return new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(0, 0.2),
            RobotContainer.Winch.goToPosition(18410, 200),
            RobotContainer.Wrist.goToPosition(0.7, .003)
        );
    }
    public static Command offGround(){
        return new ParallelCommandGroup(
            RobotContainer.Extender.goToPosition(0, 1),
            RobotContainer.Winch.goToPosition(30000, 200),
            RobotContainer.Wrist.goToPosition(0.679, 0.006)
        );
    }

}//38000 cube off ground
//12400 and wrist straight out for station pick
