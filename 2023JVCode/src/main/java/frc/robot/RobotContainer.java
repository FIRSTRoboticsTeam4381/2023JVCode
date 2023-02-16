// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Subsystems */
  Gripper Gripper;
  Extender Extender;
  LiftArm liftSystem;
  Wrist wrist;
  Balance balanceRobot;
  Medic m;

  /* Controllers */
  private final CommandPS4Controller driveController = new CommandPS4Controller(0);
  private final CommandPS4Controller specialsController = new CommandPS4Controller(1);

  /* Driver Buttons */
  private final Trigger zeroSwerve = driveController.options();
  
  /* Swerve Subsystem */
  public static final Swerve s_Swerve = new Swerve();

  //Auto Chooser
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Gripper = new Gripper();
    Extender = new Extender();
    liftSystem = new LiftArm(); // Pivots - Maybe change name
    wrist = new Wrist();
    balanceRobot = new Balance(s_Swerve); // Balancing in auto

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driveController, true));
    
    
    // Configure the button bindings
    configureButtonBindings();
    
    //Add autonoumous options to chooser
    m_AutoChooser.setDefaultOption("None", Autos.none());
    m_AutoChooser.addOption("PathPlanner Example", Autos.exampleAuto());

    SmartDashboard.putData(m_AutoChooser);
    SmartDashboard.putData("Balance Robot", balanceRobot);

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(Gripper);
    SmartDashboard.putData(Extender);
    SmartDashboard.putData(liftSystem);
    SmartDashboard.putData(wrist);

    m = new Medic(s_Swerve);
    SmartDashboard.putData("Medic!", m);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@linka
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Grabbing
    specialsController.R1().onTrue(Gripper.toggleGripperCone());
    specialsController.L1().onTrue(Gripper.toggleGripperCube());

    // Extender joystick
    Extender.setDefaultCommand(Extender.extenderJoystick(specialsController::getRightY));
    
    // Winch angle
    liftSystem.setDefaultCommand(liftSystem.winchJoystick(specialsController::getLeftY));

    // Wrist/Knuckle Angle
    wrist.setDefaultCommand(wrist.wristJoystick(specialsController::getL2Axis, specialsController::getR2Axis));
    //specialsController.L2().whileTrue(new StartEndCommand (() -> Gripper.ControledGrab(true), ()-> Gripper.ControledGrab(false)));
    //specialsController.R2().whileTrue(new StartEndCommand (() -> Gripper.ControledClose(true), ()-> Gripper.ControledClose(false)));
    
    //specialsController.R1().whileTrue(new StartEndCommand(() -> Extender.ExtendOut(true), ()-> Extender.ExtendOut(false)));
    //specialsController.L1().whileTrue(new StartEndCommand(() -> Extender.ExtendIn(true), ()-> Extender.ExtendIn(false)));

    //specialsController.povUp().whileTrue(liftSystem.armPivotPosition(0.5));
    //specialsController.povDown().whileTrue(liftSystem.armPivotPosition(-0.5));
    
    //specialsController.povUp().whileTrue(wrist.wristPivotPosition(0.2));
    //specialsController.povDown().whileTrue(wrist.wristPivotPosition(-0.2));

    
    //Button to reset swerve odometry and angle
    zeroSwerve
      .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(0))
      .alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_AutoChooser.getSelected();
  }
}
