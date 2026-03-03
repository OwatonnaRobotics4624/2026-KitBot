// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.PersistMode; // use as its not depracated
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.ResetMode; // use as its not depracated
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

import java.util.function.BooleanSupplier;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkFlex launcherRoller;
  private final SparkMax intakeRoller;


  private final SparkClosedLoopController launcherController;

  private double launcherVelocity;
  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    launcherRoller = new SparkFlex(CanIds.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    intakeRoller = new SparkMax(CanIds.INTAKE_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(CanIds.FEEDER_MOTOR_ID, MotorType.kBrushless);

    launcherController = launcherRoller.getClosedLoopController();


    
    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(CurrentLimits.FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkFlexConfig launcherConfig = new SparkFlexConfig();
    launcherConfig.inverted(true);
    //launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.closedLoop
    .p(LauncherConstants.LAUNCHER_kP)
    .i(LauncherConstants.LAUNCHER_kI)
    .d(LauncherConstants.LAUNCHER_kD)
    .outputRange(LauncherConstants.LAUNCHER_kMinOutput, LauncherConstants.LAUNCHER_kMaxOutput);
    
    launcherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the intake roller, set a current limit
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(CurrentLimits.INTAKE_MOTOR_CURRENT_LIMIT);
    intakeConfig.inverted(true);
    intakeConfig.idleMode(IdleMode.kCoast);
    intakeRoller.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching intake roller value", LAUNCHING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller RPM", LAUNCHING_LAUNCHER_RPM);
    SmartDashboard.putNumber("Slow Launching launcher roller RPM", SLOW_LAUNCHING_LAUNCHER_RPM);
    launcherVelocity = 0;
    stop();
  }

  // A method to set the ---->RPM<---- of the launcher roller NOT VOLTAGE
  public void setLauncherRollerRPM(double RPM) {
    launcherVelocity = MathUtil.clamp(RPM, LauncherConstants.LAUNCHER_MIN_RPM, LauncherConstants.LAUNCHER_MAX_RPM);
    launcherController.setSetpoint(MathUtil.clamp(launcherVelocity*2,LauncherConstants.LAUNCHER_MIN_RPM, LauncherConstants.LAUNCHER_MAX_RPM), ControlType.kVelocity);
  }

  public boolean getLauncherAtSetpoint() {
    return launcherController.isAtSetpoint();
  }

  public double getLauncherVelocity(){
    return launcherRoller.getEncoder().getVelocity();
  }

  public boolean getLauncherAtVelocity(){
    return getLauncherVelocity() >= launcherVelocity-200;
  }


  public void stopLauncher(){
    launcherVelocity = 0;
    launcherRoller.set(0);
    launcherController.setSetpoint(0, ControlType.kVoltage);
  }

  // A method to set the voltage of the launcher roller
  public void setIntakeRoller(double voltage) {
    intakeRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the feeder roller
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(voltage);
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    launcherRoller.set(0);
    intakeRoller.set(0);
    stopLauncher();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Actual Speed", getLauncherVelocity());
  }
}
