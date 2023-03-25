// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  public static Gripper Gripper;
  public static Extender Extender;
  public static Winch Winch;
  public static Wrist Wrist;
  public static Balance balanceRobot;
  public static UpRamp upRamp;
  public static LEDs leds;
  public static Limelight lime;
  public static SparkMaxPosition PIDTest;

  public static PowerDistribution pdp;

  public static FunctionalCommand forceRetractWinch;
  

  /* Controllers */
  public static final CommandPS4Controller driveController = new CommandPS4Controller(0);
  public static final CommandPS4Controller specialsController = new CommandPS4Controller(1);

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
    Winch = new Winch(); // Arm Winch/Arm Pivot
    Wrist = new Wrist();
    balanceRobot = new Balance(s_Swerve); // Balancing in auto
    upRamp = new UpRamp(s_Swerve);
    leds = new LEDs();
    pdp = new PowerDistribution(1, ModuleType.kRev);
    lime = new Limelight();

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driveController, true));
    
    // Configure the button bindings
    configureButtonBindings();
    
    //Add autonoumous options to chooser
    m_AutoChooser.setDefaultOption("None", Autos.none());
    
    //m_AutoChooser.addOption("PathPlanner Example", Autos.exampleAuto());
    //m_AutoChooser.addOption("TopPlacementAuto", Autos.TopPlacementAuto());

    m_AutoChooser.addOption("1 Place & Balance", Autos.TopPlacementBalance());
    m_AutoChooser.addOption("2 Back Cube Pickup BLUE", Autos.BackCubePickupBlue());

    m_AutoChooser.addOption("3 Back Cone Cube Pickup", Autos.BackConeCubePickup());

    //m_AutoChooser.addOption("PlaceCube&Cone", Autos.PlaceCubeandCone());
        
    m_AutoChooser.addOption("4 Front Cube Reverse", Autos.FrontCubeReverse());
    m_AutoChooser.addOption("5 Front Cone Reverse", Autos.FrontConeReverse());
    m_AutoChooser.addOption("6 Back Cube Reverse", Autos.BackCubeReverse());
    m_AutoChooser.addOption("7 Back Cone Reverse", Autos.BackConeReverse());
    
    m_AutoChooser.addOption("8 Place Only", new SequentialCommandGroup(
      SpecialistPositions.topPlacement(),
      Gripper.cubeGripper(),
      new WaitCommand(0.5),
      SpecialistPositions.zero()
    ));
    m_AutoChooser.addOption("95 Front Vision Pickup", Autos.FrontVisionPickUp());
    m_AutoChooser.addOption("96 Back Vision Pickup", Autos.BackVisionPickup());
    m_AutoChooser.addOption("97 Back Line Pickup", Autos.BackLinePickup());
    m_AutoChooser.addOption("98 Back Cube Untested", Autos.BackCubePickup());
    m_AutoChooser.addOption("99 BetterBalance", new SequentialCommandGroup(
      Autos.eventMap.get("placeTop"),
      SpecialistPositions.zero(),
      upRamp,
      upRamp.overAndBack(),
      Autos.eventMap.get("lowerArm"),
      balanceRobot
      ));
    //m_AutoChooser.addOption("Cube Over Balance", Autos.CubeOverBalance());
    


    SmartDashboard.putData(m_AutoChooser);
    SmartDashboard.putData("Balance Robot", balanceRobot);
  


    SmartDashboard.putData(Gripper);
    SmartDashboard.putData(Wrist);
    SmartDashboard.putData(Extender);
    SmartDashboard.putData("Winch Sub", Winch);
    SmartDashboard.putData(s_Swerve);

    SmartDashboard.putData(CommandScheduler.getInstance());

    SmartDashboard.putData(pdp);

    SmartDashboard.putNumber("red", 0);
    SmartDashboard.putNumber("green", 0);
    SmartDashboard.putNumber("blue", 0);

    SmartDashboard.putData("set colors", new InstantCommand( () -> {
      leds.setColors(SmartDashboard.getNumber("red", 0), SmartDashboard.getNumber("green", 0), SmartDashboard.getNumber("blue", 0));
    }));
    PIDTest = Wrist.goToPosition(0.3, 0.003);
    SmartDashboard.putData("PIDTester", PIDTest);

    forceRetractWinch = Winch.WinchResetOverride();
    SmartDashboard.putData("Force Retract Winch", forceRetractWinch);

    SmartDashboard.putData("Toggle Debug Dashboards", LogOrDash.toggleDashboard());

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@linka
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    specialsController.L1().onTrue(Gripper.cubeGripper());
    specialsController.R1().onTrue(Gripper.coneGripper());
    specialsController.circle().onTrue(SpecialistPositions.topPlacement());
    specialsController.cross().onTrue(SpecialistPositions.midPlacement());
    specialsController.share().onTrue(SpecialistPositions.zero());
    specialsController.povRight().onTrue(SpecialistPositions.autoGrabCone());
    specialsController.povLeft().onTrue(SpecialistPositions.autoGrabCube());
    specialsController.povDown().onTrue(SpecialistPositions.offGround());
    specialsController.square().onTrue(leds.setColorsCommand(0.8, 0, 1));
    specialsController.triangle().onTrue(leds.setColorsCommand(1, 0.12, 0));
    specialsController.options().onTrue(leds.setColorsCommand(0, 0, 0));
    Extender.setDefaultCommand(Extender.JoystickElevator(specialsController::getRightY));

    Winch.setDefaultCommand(Winch.JoystickWinch(specialsController::getLeftY, specialsController.getHID()::getL3Button));
    Wrist.setDefaultCommand(Wrist.JoystickWrist(specialsController::getL2Axis, specialsController::getR2Axis));
    /* OLD CONTROLS - Don't go beyond*/

    //specialsController.L1().whileTrue(new StartEndCommand (() -> Gripper.ControledGrab(true), ()-> Gripper.ControledGrab(false))); Old - for driver controlled grabbing
    //specialsController.R2().whileTrue(new StartEndCommand (() -> Gripper.ControledClose(true), ()-> Gripper.ControledClose(false))); - ^
    
    // specialsController.R1().whileTrue(new StartEndCommand(() -> Extender.ExtendOut(true), ()-> Extender.ExtendOut(false)));
    // specialsController.L1().whileTrue(new StartEndCommand(() -> Extender.ExtendIn(true), ()-> Extender.ExtendIn(false)));

    // specialsController.povUp().whileTrue(Winch.armPivotPosition(0.5));
    // specialsController.povDown().whileTrue(Winch.armPivotPosition(-0.5));
    // specialsController.R2().whileTrue(Wrist.wristPivotPosition(0.2));
    // specialsController.L2().whileTrue(Wrist.wristPivotPosition(-0.2));

    /* Now you can go you are safe */
    
    //Button to reset swerve odometry and angle
    zeroSwerve
      .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(0))
      .alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));

    driveController.share().onTrue(new InstantCommand(() -> s_Swerve.resetToCANCoders()));

    driveController.povDown().onTrue(lime.toggleCameraMode());
    // Reset scheduled commands that may be stuck
    //CommandScheduler cs = CommandScheduler.getInstance();
    specialsController.PS().onTrue(new InstantCommand(() -> {
      try
      {
        Extender.getCurrentCommand().cancel();
        //Gripper.getCurrentCommand().cancel();
        Winch.getCurrentCommand().cancel();
        Wrist.getCurrentCommand().cancel();
      }
      catch(NullPointerException e)
      {
        // This is OK, getCurrentCommand() will be null if there isn't anything running
      }
    }));
    driveController.square().whileTrue(new VisionLineup(s_Swerve, lime, leds, 1, true));
    driveController.triangle().whileTrue(new VisionLineup(s_Swerve, lime, leds, 0, false));
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
