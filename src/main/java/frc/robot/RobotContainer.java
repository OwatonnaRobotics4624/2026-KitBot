// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.FuelConstants.SLOW_LAUNCHING_LAUNCHER_RPM;
import static frc.robot.Constants.OperatorConstants.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.Jogger;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.SpinUpFlywheel;
import frc.robot.commands.StopFuelSystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;


import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    initilizeNamedCommands();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    
    autoChooser.setDefaultOption("No Autonomous", new InstantCommand());
    autoChooser.addOption("Backup And Shoot", new PathPlannerAuto("Backup And Shoot"));
    //autoChooser.addOption("Wave auto", new PathPlannerAuto("wave"));
    //autoChooser.addOption("square auto", new PathPlannerAuto("square"));


    autoChooser.addOption("Left Trench Auto",  new PathPlannerAuto("Left Trench Auto"));
    autoChooser.addOption("Right Trench Auto",  new PathPlannerAuto("Right Trench Auto"));
    autoChooser.addOption("Left Bump Auto", new PathPlannerAuto("Left Bump Auto"));
    autoChooser.addOption("Right Bump Auto", new PathPlannerAuto("Right Bump Auto"));

    autoChooser.addOption("Middle Auto Outpost",  new PathPlannerAuto("Middle Auto Outpost"));
    autoChooser.addOption("Middle Auto Straight on Depot", new PathPlannerAuto("Middle Auto Straight on Depot"));
    autoChooser.addOption("Middle Auto Diagonal on Depot", new PathPlannerAuto("Middle Auto Diagonal on Depot"));

    //autoChooser.addOption("Drive Forward auto", new PathPlannerAuto("drive forward"));

    

    SmartDashboard.putData(autoChooser);
    //SmartDashboard.setPersistent("Auto Chooser");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operatorController.a().whileTrue(new Eject(fuelSubsystem));
    //While the B button is held on the operator controller, joggle the fuel back into the hopper
    operatorController.b().whileTrue(new Jogger(fuelSubsystem));
    //While the Right Trigger is held on the operator controller, the shooter slowly shoots the ball 
    operatorController.rightTrigger().whileTrue(new LaunchSequence(
      fuelSubsystem,
      SmartDashboard.getNumber("Slow Launching launcher roller RPM", SLOW_LAUNCHING_LAUNCHER_RPM)));

    operatorController.x().whileTrue((new InstantCommand(() -> {
      driveSubsystem.resetDriverCentricYaw();
      System.out.println("Reset Yaw");
    })));


    //operatorController.b().whileTrue(getAutonomousCommand())

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
    
    RobotModeTriggers.disabled()
      .onTrue(new InstantCommand(driveSubsystem::setDriveIdleToBrake, driveSubsystem)
      .ignoringDisable(true));

    RobotModeTriggers.teleop()
      .onTrue(new InstantCommand(driveSubsystem::setDriveIdleToCoast, driveSubsystem)
      .ignoringDisable(true));
  }


  private void initilizeNamedCommands(){
    NamedCommands.registerCommand("Normal Shoot", new LaunchSequence(fuelSubsystem));
    //NamedCommands.registerCommand("Normal Shoot Timeout", new LaunchSequence(fuelSubsystem).withTimeout(5));
    NamedCommands.registerCommand("Normal Intake", new Intake(fuelSubsystem));
    //NamedCommands.registerCommand("Normal Intake Timeout", new Intake(fuelSubsystem).withTimeout(5));
    NamedCommands.registerCommand("Normal Jogger", new Jogger(fuelSubsystem));
    //NamedCommands.registerCommand("Normal Jogger", new Jogger(fuelSubsystem));
    NamedCommands.registerCommand("Stop Fuel Subsystem", new StopFuelSystem(fuelSubsystem));
    NamedCommands.registerCommand("Pre-speed up flywheel", new SpinUpFlywheel(fuelSubsystem, () -> Constants.FuelConstants.LAUNCHING_LAUNCHER_RPM));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
