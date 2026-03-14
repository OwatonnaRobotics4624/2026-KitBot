// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.SLOW_SPEED_SCALE;
import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  CommandXboxController controller;

  public Drive(CANDriveSubsystem driveSystem, CommandXboxController driverController) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    controller = driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Slow speed scale", SLOW_SPEED_SCALE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // The Y axis of the controller is inverted so that pushing the
  // stick away from you (a negative value) drives the robot forwards (a positive
  // value). The X axis is scaled down so the rotation is more easily
  // controllable.
  @Override
  public void execute() {
    double scale = SmartDashboard.getNumber("Slow speed scale", SLOW_SPEED_SCALE);
    double slowScaling = 1 - controller.getLeftTriggerAxis();
    slowScaling = slowScaling*(scale) +(1-scale);
    //System.out.println(slowScaling);
    driveSubsystem.driveArcade(
      applyDeadzone(-controller.getLeftY(), DriveConstants.DEADZONE) * DRIVE_SCALING * slowScaling,
      applyDeadzone(-controller.getRightX(), DriveConstants.DEADZONE) * ROTATION_SCALING * slowScaling,
      () -> controller.y().getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) {
      return 0.0;
    }
    // Scale input to start smoothly from 0 after the deadzone
    return Math.signum(input) * ((Math.abs(input) - deadzone) / (1.0 - deadzone));
}
}
