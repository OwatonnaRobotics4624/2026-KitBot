// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.DriveConstants.GYRO_CAN_ID;
import static frc.robot.Constants.DriveConstants.LEFT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.LEFT_LEADER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_LEADER_ID;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
//import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.PersistMode; // use as its not depracated
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.ResetMode; // use as its not depracated
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.OdometryConstants;
import frc.robot.Constants.DriveConstants.SysIdConstants;
import frc.robot.LimelightHelpers;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  //private final DifferentialDrive drive;

  private SparkMaxConfig leftDriveLeaderConfig;
  private SparkMaxConfig rightDriveLeaderConfig;
  private SparkMaxConfig leftDriveFollowerConfig;
  private SparkMaxConfig rightDriveFollowerConfig;
  
  private EncoderConfig m_leftEncoderConfig;
  private EncoderConfig m_rightEncoderConfig;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  private SparkClosedLoopController m_leftController;
  private SparkClosedLoopController m_rightController;

  /*uncomment once you actually have sysid values */
  
  /* 
  private DifferentialDriveFeedforward m_feedforward = new DifferentialDriveFeedforward(
    SysIdConstants.kVLinear,
    SysIdConstants.kALinear,
    SysIdConstants.kVAngular,
    SysIdConstants.kAAngular,
    OdometryConstants.TRACK_WIDTH_METERS
  );
  */

  //private DifferentialDriveOdometry m_odometry;

  private DifferentialDriveKinematics m_kinematics;
  private ChassisSpeeds chassisSpeeds;
  private DifferentialDriveWheelSpeeds wheelSpeeds;

  private double linearVelocity;
  private double angularVelocity;

  private Pigeon2 m_gyro;

  private Pose2d m_robotPose;

  private Field2d field2d; 

  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private SysIdRoutine m_sysIdRoutine = null;
  
    public CANDriveSubsystem() {
  
  
      
      // create brushed motors for drive
      leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
      leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
      rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
      rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);
  
      
  
      //Accesses Smart Dashboard and Sets Initial Robot Speed
      SmartDashboard.putNumber("Robot Speed", Constants.DriveConstants.DEFAULT_ROBOT_SPEED);
      SmartDashboard.setPersistent("Robot Speed");
  
      
      // set up differential drive class
      //drive = new DifferentialDrive(leftLeader, rightLeader);
  
      // Set can timeout. Because this project only sets parameters once on
      // construction, the timeout can be long without blocking robot operation. Code
      // which sets or gets parameters during operation may need a shorter timeout.
      leftLeader.setCANTimeout(250);
      rightLeader.setCANTimeout(250);
      leftFollower.setCANTimeout(250);
      rightFollower.setCANTimeout(250);
  
      // Create the configuration to apply to motors. Voltage compensation
      // helps the robot perform more similarly on different
      // battery voltages (at the cost of a little bit of top speed on a fully charged
      // battery). The current limit helps prevent tripping
      // breakers.
  
      leftDriveLeaderConfig = new SparkMaxConfig();
      rightDriveLeaderConfig = new SparkMaxConfig();
      leftDriveFollowerConfig = new SparkMaxConfig();
      rightDriveFollowerConfig = new SparkMaxConfig();
  
      ClosedLoopConfig leftPIDConfig = new ClosedLoopConfig();
      ClosedLoopConfig rightPIDConfig = new ClosedLoopConfig();
  
      leftPIDConfig.pid(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD);
      rightPIDConfig.pid(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD);
  
      leftDriveLeaderConfig.closedLoop.apply(leftPIDConfig);
      rightDriveLeaderConfig.closedLoop.apply(rightPIDConfig);
  
      // Set configuration to follow each leader and then apply it to corresponding
      // follower. Resetting in case a new controller is swapped
      // in and persisting in case of a controller reset due to breaker trip
  
      leftDriveLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
      rightDriveLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
      leftDriveFollowerConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
      rightDriveFollowerConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
  
      m_leftEncoderConfig = new EncoderConfig();
      m_rightEncoderConfig = new EncoderConfig();
  
      m_leftEncoderConfig.positionConversionFactor(OdometryConstants.kPositionFactor);
      m_rightEncoderConfig.positionConversionFactor(OdometryConstants.kPositionFactor);
  
      m_leftEncoderConfig.velocityConversionFactor(OdometryConstants.kVelocityFactor);
      m_rightEncoderConfig.velocityConversionFactor(OdometryConstants.kVelocityFactor);
  
      leftDriveLeaderConfig.encoder.apply(m_leftEncoderConfig);
      rightDriveLeaderConfig.encoder.apply(m_rightEncoderConfig);
      leftDriveFollowerConfig.encoder.apply(m_leftEncoderConfig);
      rightDriveFollowerConfig.encoder.apply(m_rightEncoderConfig);
      
  
      //left follower (not inverted, on coast)
      leftDriveFollowerConfig.follow(leftLeader);
      leftDriveFollowerConfig.inverted(false);
      leftDriveFollowerConfig.idleMode(DriveConstants.kEnabledIdle);
      leftFollower.configure(leftDriveFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  
      //right follower (inverted, on coast)
      rightDriveFollowerConfig.follow(rightLeader);
      rightDriveFollowerConfig.inverted(true);
      rightDriveFollowerConfig.idleMode(DriveConstants.kEnabledIdle);
      rightFollower.configure(rightDriveFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      //left leader (not inverted, on coast)
      leftDriveLeaderConfig.inverted(false);
      leftDriveLeaderConfig.idleMode(DriveConstants.kEnabledIdle);
      leftLeader.configure(leftDriveLeaderConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      //right leader (inverted, on coast)
      rightDriveLeaderConfig.inverted(true);
      rightDriveLeaderConfig.idleMode(DriveConstants.kEnabledIdle);
      rightLeader.configure(rightDriveLeaderConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
      m_leftController = leftLeader.getClosedLoopController();
      m_rightController = rightLeader.getClosedLoopController();
      
  
      m_gyro = new Pigeon2(GYRO_CAN_ID);
      
      m_leftEncoder = leftLeader.getEncoder();
      m_rightEncoder = rightLeader.getEncoder();
  
      field2d = new Field2d();  
  
      // Creating my kinematics object: track width of 55 cm
      m_kinematics = new DifferentialDriveKinematics(0.546);
      wheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
      // Convert to chassis speeds.
      chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);
      // Linear velocity
      linearVelocity = chassisSpeeds.vxMetersPerSecond;
      // Angular velocity
      angularVelocity = chassisSpeeds.omegaRadiansPerSecond;
  
      m_poseEstimator =
      new DifferentialDrivePoseEstimator(
          m_kinematics,
          m_gyro.getRotation2d(),
          m_leftEncoder.getPosition(),
          m_rightEncoder.getPosition(),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(30))
      );
  
      setUpPathplanner();
      setUpLimelights();

      //WARNING: IDK what the f**k this does this was spit out of chatgpt.
      if(SysIdConstants.RunningSysIDTuning_TURN_THIS_OFF){
        m_sysIdRoutine =
          new SysIdRoutine(
            new SysIdRoutine.Config(
              Volts.per(Seconds).of(1),
              Volts.of(7),
              Seconds.of(10)
            ),

            new SysIdRoutine.Mechanism(
              (Voltage volts) -> {
                  leftLeader.setVoltage(volts.in(Volts));
                  rightLeader.setVoltage(volts.in(Volts));
              },

              log -> {
                log.motor("left-drive")
                  .voltage(Volts.of(leftLeader.getBusVoltage() * leftLeader.getAppliedOutput()))
                  .linearPosition(edu.wpi.first.units.Units.Meters.of(m_leftEncoder.getPosition()))
                  .linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(m_leftEncoder.getVelocity()));

                log.motor("right-drive")
                  .voltage(Volts.of(rightLeader.getBusVoltage() * rightLeader.getAppliedOutput()))
                  .linearPosition(edu.wpi.first.units.Units.Meters.of(m_rightEncoder.getPosition()))
                  .linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(m_rightEncoder.getVelocity()));
              },
              this
            )
          );
    }

  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    if (m_sysIdRoutine == null) {
      throw new IllegalStateException("Hey if you are done running sysid Go delete all of the sysid junk");
    }
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    if (m_sysIdRoutine == null) {
      throw new IllegalStateException("Hey if you are done running sysid Go delete all of the sysid junk");
    }
    return m_sysIdRoutine.dynamic(direction);
  }

  private void setUpLimelights(){
    LimelightHelpers.setPipelineIndex("limelight-bl", 0);
    LimelightHelpers.setLEDMode_ForceOff("limelight-bl");

    LimelightHelpers.setPipelineIndex("limelight-fr", 0);
    LimelightHelpers.setLEDMode_ForceOff("limelight-fr");
  }

  private void setUpPathplanner(){
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = Constants.PathPlannerConstants.ROBOT_CONFIG;

    // Configure AutoBuilder last
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  
  

  @Override
  public void periodic() {
    updateOdometry();
    updateKinematics();
    updatePoseEstimator();
    updatePoseVisionEstimator();

  }

  private void updatePoseVisionEstimator(){
    //Boy-love limelight (jk back left)

    double robotYaw = m_robotPose.getRotation().getDegrees();
    double robotPitch = m_gyro.getPitch().getValueAsDouble();
    double robotRoll = m_gyro.getRoll().getValueAsDouble();


    if (Constants.VisionConstants.USE_LIMELIGHT_BL){
      if (Constants.VisionConstants.USE_MEGA_TAG_1) {
        // In your periodic function:
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-bl");
        if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 30));
          m_poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds
          );
        }
      }

      if (Constants.VisionConstants.USE_MEGA_TAG_2) {
        
        // First, tell Limelight your robot's current orientation
        LimelightHelpers.SetRobotOrientation("limelight-bl", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bl");
        if (limelightMeasurement.tagCount >= 1) {  // Only trust measurement if we see multiple tags
          // Add it to your pose estimator
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.2, .2, 30));
          m_poseEstimator.addVisionMeasurement(
              limelightMeasurement.pose,
              limelightMeasurement.timestampSeconds
          );
        }
      }
    }

    //Front right limelight
    if (Constants.VisionConstants.USE_LIMELIGHT_FR){
      if (Constants.VisionConstants.USE_MEGA_TAG_1) {
        // In your periodic function:
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fr");
        if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 30));
          m_poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds
          );
        }
      }

      if (Constants.VisionConstants.USE_MEGA_TAG_2) {
        // First, tell Limelight your robot's current orientation
        LimelightHelpers.SetRobotOrientation("limelight-fr", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-fr");
        if (limelightMeasurement.tagCount >= 1) {  // Only trust measurement if we see multiple tags
          // Add it to your pose estimator
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.2, .2, 30));
          m_poseEstimator.addVisionMeasurement(
              limelightMeasurement.pose,
              limelightMeasurement.timestampSeconds
          );
        }
      }
    }
  }

  private void updatePoseEstimator() {
    m_poseEstimator.update(
      m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  private void updateOdometry(){
    // Get the rotation of the robot from the gyro.
    // Update the pose
    m_robotPose = m_poseEstimator.getEstimatedPosition();
    
    field2d.setRobotPose(m_robotPose);
    
    SmartDashboard.putData("Field", field2d);
  }

  private void updateKinematics(){
    wheelSpeeds = new DifferentialDriveWheelSpeeds(
      m_leftEncoder.getVelocity(),
      m_rightEncoder.getVelocity()
    );
    // Convert to chassis speeds.
    chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);
    // Linear velocity
    linearVelocity = chassisSpeeds.vxMetersPerSecond;
    // Angular velocity
    angularVelocity = chassisSpeeds.omegaRadiansPerSecond;

    SmartDashboard.putNumber("Linear Velocity (m/s)", Math.abs(linearVelocity) );
    SmartDashboard.putNumber("Angular Velocity (rad/s)", Math.abs(angularVelocity));
    
    SmartDashboard.putNumber("Left Distance", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Distance", m_rightEncoder.getPosition());

    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getVelocity());

  }

  public Pose2d getPose(){
    return m_robotPose;
  }

  public Field2d getField2d(){
    return field2d;
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return chassisSpeeds;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return wheelSpeeds;
  }

  //Meters Per Second
  public double getLinearVelocity(){
    return linearVelocity;
  }

  //Radians Per Second
  public double getAngularVelocity(){
    return angularVelocity;
  }

  public void resetPose(Pose2d newPose){
    m_poseEstimator.resetPose(newPose);
    m_gyro.setYaw(newPose.getRotation().getDegrees());
  }

  public void resetDriverCentricYaw(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red){
        m_gyro.setYaw(180);
        
      } else {
        m_gyro.setYaw(0);
      }
    }else{
      m_gyro.setYaw(0);
    }
  }

  public void resetYaw(){
    m_gyro.reset();
  }

  public void resetPosition(Rotation2d gyroAngle, DifferentialDriveWheelPositions wheelPositions, Pose2d poseMeters){
    m_poseEstimator.resetPosition(gyroAngle,  wheelPositions, poseMeters);
  }

  public void resetPosition(Rotation2d gyroAngle, double leftDistanceMeters, double rightDistanceMeters, Pose2d poseMeters){
    m_poseEstimator.resetPosition(gyroAngle, leftDistanceMeters, rightDistanceMeters, poseMeters);
  }

  public Rotation2d gyroRotation2d(){
    return m_gyro.getRotation2d();
  }

  public void driveArcade(double xSpeed, double zRotation) {
    driveArcade(xSpeed, zRotation, false);
  }

  public void driveArcade(double xSpeed, double zRotation, BooleanSupplier driveAtMax) {
    driveArcade(xSpeed, zRotation, driveAtMax.getAsBoolean());
  }

  public void driveArcade(double xSpeed, double zRotation, boolean driveAtMax) {
    double speedMultiplier = driveAtMax
        ? 1.0
        : SmartDashboard.getNumber("Robot Speed", Constants.DriveConstants.DEFAULT_ROBOT_SPEED);

    speedMultiplier = MathUtil.clamp(speedMultiplier, 0.0, 1.0);
    SmartDashboard.putNumber("Robot Speed", speedMultiplier);

    double leftOutput = xSpeed + zRotation;
    double rightOutput = xSpeed - zRotation;

    double maxMagnitude = Math.max(1.0,
        Math.max(Math.abs(leftOutput), Math.abs(rightOutput)));

    leftOutput /= maxMagnitude;
    rightOutput /= maxMagnitude;

    leftLeader.set(leftOutput * speedMultiplier);
    rightLeader.set(rightOutput * speedMultiplier);
  }


  private void driveRobotRelative(ChassisSpeeds speeds) {

    DifferentialDriveWheelSpeeds wheelSpeeds =
        m_kinematics.toWheelSpeeds(speeds);

    m_leftController.setSetpoint(
        wheelSpeeds.leftMetersPerSecond,
        ControlType.kVelocity
    );

    m_rightController.setSetpoint(
        wheelSpeeds.rightMetersPerSecond,
        ControlType.kVelocity
    );
  }

  /* 
  private void driveRobotRelative(ChassisSpeeds speeds) {

    DifferentialDriveWheelSpeeds targetSpeeds =
        m_kinematics.toWheelSpeeds(speeds);

    DifferentialDriveWheelVoltages ff = feedforward.calculate(
        getWheelSpeeds(),
        targetSpeeds
    );

    leftController.setSetpoint(
        targetSpeeds.leftMetersPerSecond,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ff.leftVoltage,
        ArbFFUnits.kVoltage
    );

    rightController.setSetpoint(
        targetSpeeds.rightMetersPerSecond,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ff.rightVoltage,
        ArbFFUnits.kVoltage
    );
  }*/

  public void setDriveIdleToBrake(){
    setAllDriveIdleMode(IdleMode.kBrake, PersistMode.kNoPersistParameters);
  }

  public void setDriveIdleToCoast(){
    setAllDriveIdleMode(IdleMode.kCoast, PersistMode.kNoPersistParameters);
  }

  public void setDriveIdleMode(IdleMode idleMode, PersistMode persistMode){
    setAllDriveIdleMode(idleMode, persistMode);
  }

  public void setDriveIdleMode(IdleMode idleMode){
    setDriveIdleMode(idleMode, PersistMode.kNoPersistParameters);
  }



  private void setAllDriveIdleMode(IdleMode idleMode, PersistMode persistMode){
    leftDriveLeaderConfig.idleMode(idleMode);
    rightDriveLeaderConfig.idleMode(idleMode);
    leftDriveFollowerConfig.idleMode(idleMode);
    rightDriveFollowerConfig.idleMode(idleMode);
    leftFollower.configure(leftDriveFollowerConfig, ResetMode.kResetSafeParameters, persistMode);
    rightFollower.configure(rightDriveFollowerConfig, ResetMode.kResetSafeParameters, persistMode);
    leftLeader.configure(leftDriveLeaderConfig,ResetMode.kResetSafeParameters, persistMode);
    rightLeader.configure(rightDriveLeaderConfig,ResetMode.kResetSafeParameters, persistMode);
  }

  


}








