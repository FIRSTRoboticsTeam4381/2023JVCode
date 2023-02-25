// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  CANifier canifier;
  /** Creates a new LEDs. */
  public LEDs() {
    canifier = new CANifier(9);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setColors(double red, double green, double blue){
    canifier.setLEDOutput(red, LEDChannel.LEDChannelA);
    canifier.setLEDOutput(green, LEDChannel.LEDChannelB);
    canifier.setLEDOutput(blue, LEDChannel.LEDChannelC);
  }
  public InstantCommand setColorsCommand(double red, double green, double blue){
    return new InstantCommand(() -> setColors(red, green, blue));
  }
}
