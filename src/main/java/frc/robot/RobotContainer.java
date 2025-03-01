// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbAngle;
import frc.robot.commands.ClimbMove;
import frc.robot.commands.ClimbReset;
import frc.robot.commands.ClimbAngle.ClimbPosition;
import frc.robot.commands.swervedrive.auto.PivotDealgaenatorToAngle;
import frc.robot.commands.swervedrive.auto.PivotIntakeToAngle;
import frc.robot.commands.swervedrive.auto.ResetDealgaenator;
import frc.robot.commands.swervedrive.auto.ResetPivot;
import frc.robot.commands.swervedrive.auto.SpinDealgaenator;
import frc.robot.controller.Controller;
import frc.robot.controller.GuitarController;
import frc.robot.controller.Controller.Deadzone;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DealgaenatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DealgaenatorSubsystem.DealgaenatorPosition;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  final Controller driverController = new Controller(0)
      .invertLeftX()
      .invertLeftY()
      .setLeftDeadzone(0d);
  final GuitarController codriverController = new GuitarController(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase;
  private IntakeSubsystem intake;
  private DealgaenatorSubsystem dealgaenator;
  public ClimbSubsystem climb;
  {
    if (Robot.isReal()) {
      drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve_non_simulation"));
      drivebase.getSwerveDrive().setCosineCompensator(false);
      intake = new IntakeSubsystem();
      climb = new ClimbSubsystem();
      dealgaenator = new DealgaenatorSubsystem();
      
    } else {
      drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve_simulation"));
      drivebase.getSwerveDrive().setHeadingCorrection(false); // Heading correction should only be used while
                                                              // controlling the robot via angle.
      drivebase.getSwerveDrive().setCosineCompensator(false);
      intake = new IntakeSubsystem();
      climb = new ClimbSubsystem();
      dealgaenator = new DealgaenatorSubsystem();

    }
  }

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverController.getLeftStickX(),
      () -> driverController.getLeftStickY())
      .withControllerRotationAxis(driverController::getRightStickX)
      .deadband(OperatorConstants.DEADBAND)
      // .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverController::getRightStickX,
          driverController::getRightStickY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverController.getLeftStickY(),
      () -> -driverController.getLeftStickX())
      .withControllerRotationAxis(driverController::getRightStickX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverController.getLeftTrigger() *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driverController.getLeftTrigger() *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    codriverController.fretOrange().onTrue(new PivotIntakeToAngle(intake, IntakePosition.GROUND_INTAKE));
    codriverController.fretBlue().onTrue(new PivotIntakeToAngle(intake, IntakePosition.SCORING));
    codriverController.fretYellow().onTrue(new PivotIntakeToAngle(intake, IntakePosition.CORAL_SNAG));
    codriverController.fretRed().onTrue(new PivotIntakeToAngle(intake, IntakePosition.CLIMBING));
    codriverController.fretGreen().onTrue(new PivotIntakeToAngle(intake, IntakePosition.DEALGAENATING));

    codriverController.strumUp().onTrue(new SpinDealgaenator(dealgaenator, .7));
    codriverController.strumUp().onFalse(new SpinDealgaenator(dealgaenator, 0));
    codriverController.strumDown().onTrue(new SpinDealgaenator(dealgaenator, -.7));
    codriverController.strumDown().onFalse(new SpinDealgaenator(dealgaenator, 0));

    codriverController.buttonStart().onTrue(new ClimbMove(climb, .2d));
    codriverController.buttonStart().onFalse(new ClimbMove(climb, 0d));

    codriverController.buttonBack().onTrue(new ClimbMove(climb, -.2d));
    codriverController.buttonBack().onFalse(new ClimbMove(climb, 0d));

    codriverController.dpadLeft().onTrue(new ClimbAngle(climb, ClimbPosition.DEPLOY_CLIMB));

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else {
      // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      driverController.buttonStart()
          .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverController.buttonA().whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverController.buttonX().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverController.buttonY().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverController.buttonStart().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.buttonBack().whileTrue(drivebase.centerModulesCommand());
      driverController.leftBumper().onTrue(Commands.none());
      driverController.rightBumper().onTrue(Commands.none());
    } else {
      driverController.buttonX().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverController.buttonA().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverController.buttonX().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverController.buttonB().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driverController.buttonStart().whileTrue(Commands.none());
      driverController.buttonBack().whileTrue(Commands.none());
      driverController.leftBumper().onTrue(intake.spinIntake(Constants.INTAKE_SPEED));
      driverController.rightBumper().onTrue(intake.spinIntake(-Constants.INTAKE_SPEED));
      driverController.leftBumper().onFalse(intake.spinIntake(0));
      driverController.rightBumper().onFalse(intake.spinIntake(0));
    }

  }

  public void resetIntakePivot() {
    new ResetPivot(intake).schedule();

  }

  public Command resetRobot() {
    return Commands.sequence(
        new ResetPivot(intake), 
        new PivotIntakeToAngle(intake, IntakePosition.GROUND_INTAKE),
        Commands.parallel(
            new ResetDealgaenator(dealgaenator), 
            new ClimbReset(climb)),
        new PivotDealgaenatorToAngle(dealgaenator, DealgaenatorPosition.DEPLOYED)
        );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("main");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void sendTelementary() {
    SmartDashboard.putData("gyro", (Sendable) drivebase.getSwerveDrive().getGyro().getIMU());
    if (Robot.isSimulation()) {
      Logger.recordOutput("wheelStates", drivebase.getSwerveDrive().getStates());

    }
  }

  public void resetSimulation() {
    if (Robot.isReal())
      return;

    drivebase.getSwerveDrive().getMapleSimDrive().get().setSimulationWorldPose(new Pose2d(9.3, 2, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Robot.isReal()) {
      return;
    }

    Logger.recordOutput("FieldSimulation/RobotPosition",
        drivebase.getSwerveDrive().getMapleSimDrive().get().getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/Algae",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput("FieldSimulation/Coral",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
  }
}
