// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopFuelSystem extends Command {

  CANFuelSubsystem fuelSubsystem;
  /** Creates a new StopFuelSystem. */
  public StopFuelSystem(CANFuelSubsystem fuelSubsystem) {
    this.fuelSubsystem=fuelSubsystem;
    addRequirements(fuelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fuelSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fuelSubsystem.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
