// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import org.ejml.dense.row.decomposition.BaseDecomposition_DDRB_to_DDRM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.LogOrDash;
import frc.robot.RobotContainer;


/*
 * 
 * ============ WARNING WARNING WARNING ===============
 * 
 * ALL COMMANDS MUST BE RUN AS PROXY IN COMMAND COMPOSITIONS!
 * 
 * THIS SUBSYSTEM SCHEDULES ITS OWN COMMANDS AS A STATE MACHINE,
 * THUS CANCELLING COMMAND GROUPS IT IS A PART OF!
 * 
 */

public class RollerGripper extends SubsystemBase {
  private TalonSRX Gripper;
  public boolean gripperOn;

  private final double GRAB_CONE_POW = 1.0;
  private final double HOLD_CONE_POW = 0.2;

  private final double GRAB_CUBE_POW = 0.8;
  private final double HOLD_CUBE_POW = 0.1;

  private final double CUBE_DETECT_DIFF = 1500;
  private final double CONE_DETECT_DIFF = 3000;

  // Commands, which will be treated like a state machine
  public Command ejectCone() {

    return new SequentialCommandGroup(
      new InstantCommand(() -> Gripper.set(ControlMode.PercentOutput, -1), this),
      new WaitCommand(0.5).withName("Ejecting Cone"),
      new InstantCommand(() ->Gripper.set(ControlMode.PercentOutput, 0), this)
    );
  }

  public Command ejectCube() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> Gripper.set(ControlMode.PercentOutput, 1), this),
      new WaitCommand(0.5).withName("Ejecting Cube"),
      new InstantCommand(() ->Gripper.set(ControlMode.PercentOutput, 0), this));
  }

  // This was quite annoying to do. Apparently, a running command is always encased
  // in another object so you can't use class equality to check which command is running.
  // You also can't use a class instance because once a command is "composed" it can't
  // be reused elsewhere, so it can essentially only be called from one place.
  // Using the command's Name string avoids this problem.

  /**
   * Command which toggles gripping a cone.
   * @return The command
   */
  public Command coneGripper () {
    return new InstantCommand( () -> {
      Command current = this.getCurrentCommand();
      if(current != null)
      {
        Command c = Map.ofEntries(
        Map.entry("Gripper Idle", new GrabCone(this)),
        Map.entry("Grabbing Cone", new InstantCommand(() -> {}, this)),
        Map.entry("Grabbing Cube", new InstantCommand(() -> {}, this)),
        Map.entry("Holding Cone", ejectCone()),
        Map.entry("Holding Cube", ejectCube())
        ).get(current.getName());
        if(c != null)
         c.schedule();
      }
    });
  }

  /**
   * Command which toggles gripping a cube.
   * @return The command
   */
  public Command cubeGripper () {
    return new InstantCommand( () -> {
      Command current = this.getCurrentCommand();
      if(current != null)
      {
      Command c = Map.ofEntries(
        Map.entry("Gripper Idle", new GrabCube(this)),
        Map.entry("Grabbing Cone", new InstantCommand(() -> {}, this)),
        Map.entry("Grabbing Cube", new InstantCommand(() -> {}, this)),
        Map.entry("Holding Cone", ejectCone()),
        Map.entry("Holding Cube", ejectCube())
        ).get(current.getName());
        if(c != null)
         c.schedule();
    }
  });
  }

  /** Creates a new Gripper. */
  public RollerGripper() {
    Gripper = new TalonSRX(3);

    TalonSRXConfiguration gripperConfig = new TalonSRXConfiguration();
    gripperConfig.slot0.kP = 1;
    gripperConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    gripperConfig.clearPositionOnLimitR = false;
    gripperConfig.continuousCurrentLimit = 25;
    gripperConfig.peakCurrentLimit = 25;
    gripperConfig.peakCurrentDuration = 1;
    gripperConfig.peakOutputReverse = -1;
    Gripper.setInverted(InvertType.InvertMotorOutput);
    Gripper.configAllSettings(gripperConfig);
    Gripper.setSensorPhase(false);
    Gripper.setNeutralMode(NeutralMode.Brake);
    //Gripper.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

    this.setDefaultCommand(new IdleCommand(this).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    LogOrDash.logNumber("gripper/position", Gripper.getSelectedSensorPosition());
    LogOrDash.logBoolean("gripper/reverselimit", Gripper.isRevLimitSwitchClosed()==1);
    LogOrDash.logNumber("gripper/looperror",Gripper.getClosedLoopError());
    //SmartDashboard.putNumber("gripper/target",Gripper.getClosedLoopTarget());
    LogOrDash.logNumber("gripper/derivative",Gripper.getErrorDerivative());
    LogOrDash.logNumber("gripper/iaccum",Gripper.getIntegralAccumulator());
    LogOrDash.logNumber("gripper/outpercent",Gripper.getMotorOutputPercent());
    LogOrDash.logNumber("gripper/outvoltage",Gripper.getMotorOutputVoltage());
    LogOrDash.logNumber("gripper/velocity",Gripper.getSelectedSensorVelocity());
    LogOrDash.logNumber("gripper/statorcurrent",Gripper.getStatorCurrent());
    LogOrDash.logNumber("gripper/supplycurrent",Gripper.getSupplyCurrent());
    LogOrDash.logNumber("gripper/controllertemp",Gripper.getTemperature());

    // Fault detection
    Faults f = new Faults();
    Gripper.getFaults(f);

    if(f.hasAnyFault())
    {
      LogOrDash.logString("gripper/faults", f.toString());
    }


    // Check for reboot
    if(Gripper.hasResetOccurred())
    {
        DriverStation.reportError("ALERT: Gripper motor has crashed!", false);
    }
  }

  /*
   * Command class for grabbing a cone
   */
  public class GrabCone extends CommandBase {
    RollerGripper gripper;
    //boolean speedExceeded = false;
    double topSpeed;

    public GrabCone(RollerGripper gripper) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.gripper = gripper;
      
      addRequirements(gripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      gripper.Gripper.set(ControlMode.PercentOutput, GRAB_CONE_POW);
      //speedExceeded = false;
      topSpeed = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      /*if(!speedExceeded && gripper.Gripper.getSelectedSensorVelocity() > 1500)
      {
        speedExceeded = true;
      }*/
      double v = gripper.Gripper.getSelectedSensorVelocity();
      if(v > topSpeed)
        topSpeed = v;

      // Pulse LEDs
      double pulseLevel = (Math.sin(Timer.getFPGATimestamp()*10)+1)/2;
      RobotContainer.leds.setColors(pulseLevel, pulseLevel * 0.12, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if(interrupted)
      {
        gripper.Gripper.set(ControlMode.PercentOutput, 0);
        RobotContainer.leds.setColors(1, 0, 0);
      }
      else
      {
        // Schedule holding command
        new HoldCone(gripper).ignoringDisable(true).schedule();
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      //return speedExceeded && gripper.Gripper.getSelectedSensorVelocity() < 1000;
      return gripper.Gripper.getSelectedSensorVelocity() < topSpeed - CONE_DETECT_DIFF;
    }

    @Override
    public String getName()
    {
      return "Grabbing Cone";
    }
  }

  /*
   * Command while holding a cone
   */
  public class HoldCone extends CommandBase {
    RollerGripper gripper;
    double startTime;
    boolean greenTimeUp = false;

    public HoldCone(RollerGripper gripper) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.gripper = gripper;
      
      addRequirements(gripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      gripper.Gripper.set(ControlMode.PercentOutput, HOLD_CONE_POW);
      startTime = Timer.getFPGATimestamp();
      RobotContainer.leds.setColors(0, 1, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(greenTimeUp)
      {
        // Pulse LEDs
        double pulseLevel = Math.max((Math.sin(Timer.getFPGATimestamp()*10)+1)/2 - 0.75, 0);
        RobotContainer.leds.setColors(0, pulseLevel*0.5, pulseLevel);
      }
      else
      {
        if(Timer.getFPGATimestamp() > startTime + 1)
        {
          greenTimeUp = true;
        }
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      RobotContainer.leds.setColors(0, 0, 0);
      gripper.Gripper.set(ControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public String getName()
    {
      return "Holding Cone";
    }
  }



  /*
   * Command class for grabbing a cone
   */
  public class GrabCube extends CommandBase {
    RollerGripper gripper;
    //boolean speedExceeded = false;

    double topSpeed;

    public GrabCube(RollerGripper gripper) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.gripper = gripper;
      
      addRequirements(gripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      gripper.Gripper.set(ControlMode.PercentOutput, -GRAB_CUBE_POW);
      //speedExceeded = false;
      topSpeed = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      /*if(!speedExceeded && gripper.Gripper.getSelectedSensorVelocity() < -5000)
      {
        speedExceeded = true;
      }*/

      double v = gripper.Gripper.getSelectedSensorVelocity();
      if(v < topSpeed)
        topSpeed = v;

      // Pulse LEDs
      double pulseLevel = (Math.sin(Timer.getFPGATimestamp()*10)+1)/2;
      RobotContainer.leds.setColors(pulseLevel * 0.8, 0, pulseLevel);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if(interrupted)
      {
        gripper.Gripper.set(ControlMode.PercentOutput, 0);
        RobotContainer.leds.setColors(1, 0, 0);
      }
      else
      {
        // Schedule holding command
        new HoldCube(gripper).ignoringDisable(true).schedule();
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      //return speedExceeded && gripper.Gripper.getSelectedSensorVelocity() > -4500;
      return gripper.Gripper.getSelectedSensorVelocity() > topSpeed + CUBE_DETECT_DIFF;
    }

    @Override
    public String getName()
    {
      return "Grabbing Cube";
    }
  }

  /*
   * Command while holding a cone
   */
  public class HoldCube extends CommandBase {
    RollerGripper gripper;
    double startTime;
    boolean greenTimeUp = false;

    public HoldCube(RollerGripper gripper) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.gripper = gripper;
      
      addRequirements(gripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      gripper.Gripper.set(ControlMode.PercentOutput, -HOLD_CUBE_POW);
      startTime = Timer.getFPGATimestamp();
      RobotContainer.leds.setColors(0, 1, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(greenTimeUp)
      {
        // Pulse LEDs
        double pulseLevel = Math.max((Math.sin(Timer.getFPGATimestamp()*10)+1)/2 - 0.75, 0);
        RobotContainer.leds.setColors(0, pulseLevel*0.5, pulseLevel);
      }
      else
      {
        if(Timer.getFPGATimestamp() > startTime + 1)
        {
          greenTimeUp = true;
        }
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      RobotContainer.leds.setColors(0, 0, 0);
      gripper.Gripper.set(ControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public String getName()
    {
      return "Holding Cube";
    }
  }

  // Blank command for when idle
  // Used for polymorphic check to use this as a state machine because we can't check for null
  public class IdleCommand extends CommandBase
  {
    public IdleCommand(RollerGripper rollerGripper)
    {
      addRequirements(rollerGripper);
    }

    public String getName()
    {
      return "Gripper Idle";
    }
  }

}