// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorPIDConstants;
import frc.robot.commands.operator.ClimberCmd;
import frc.robot.commands.operator.ElevatorCmd;
import frc.robot.commands.operator.ElevatorPIDCmd;
import frc.robot.commands.operator.IndexerCmd;
import frc.robot.commands.operator.IntakeCmd;
import frc.robot.commands.operator.ShooterCmd;
import frc.robot.subsystems.operator.ClimberSubsystem;
import frc.robot.subsystems.operator.ElevatorSubsystem;
import frc.robot.subsystems.operator.IndexerSubsystem;
import frc.robot.subsystems.operator.IntakeSubsystem;
import frc.robot.subsystems.operator.ShooterSubsystem;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final XboxController operatorXbox = new XboxController(OperatorConstants.JOYSTICK_OPERATOR);
  private final XboxController driverXbox = new XboxController(DrivebaseConstants.JOYSTICK_DRIVER);
  /** 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // COMMENTED BELOW OUT!!!
    /*AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox::getYButtonPressed,
                                                                   driverXbox::getAButtonPressed,
                                                                   driverXbox::getXButtonPressed,
                                                                   driverXbox::getBButtonPressed);
    */
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation

    //final double speedMultiplier;

    /*    UNCOMMENT THIS AND ADD *speedMultiplier TO driverXbox.getLeftY() IN Command driveFieldOrientedDirectAngle BELOW
    if (driverXbox.getRawButton(DrivebaseConstants.ARCADE_DRIVE_TURBO)){
      speedMultiplier = DrivebaseConstants.DRIVE_TURBO;
    } else {
      speedMultiplier = DrivebaseConstants.DRIVE_THROTTLE;
    }
    */

    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(4));
        /*() -> driverXbox.getRightX(),
        () -> driverXbox.getRightY()); */

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    //test = MathUtil.applyDeadband((driverXbox.getLeftY()), OperatorConstants.LEFT_Y_DEADBAND);

    // COMMENTED BELOW OUT!!!!
    /*Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));
    */

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    //new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    /*new JoystickButton(driverXbox,
                       2).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              )); */
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    //intakeSubsystem.setDefaultCommand(new IntakeCmd(intakeSubsystem, () -> -operatorXbox.getRawAxis(Constants.OperatorConstants.INTAKE_AXIS)));
    new JoystickButton(operatorXbox, OperatorConstants.INTAKE_IN).onTrue(new IntakeCmd(intakeSubsystem, 1)); //intake
    new JoystickButton(operatorXbox, OperatorConstants.INTAKE_OUT).onTrue(new IntakeCmd(intakeSubsystem, -1)); // reverse intake
    //new JoystickButton(operatorXbox, OperatorConstants.SHOOTER).whileTrue(new ShooterCmd(shooterSubsystem, 1)); //intake
    new JoystickButton(operatorXbox, OperatorConstants.SHOOTER).whileTrue(Commands.parallel(new IndexerCmd(indexerSubsystem, 1), (new WaitCommand(1.0).andThen(new ShooterCmd(shooterSubsystem, 1))))); //intake
    //new JoystickButton(operatorXbox, OperatorConstants.CLIMBER_UP).onTrue(new ClimberCmd(climberSubsystem, 1)); //intake
    //new JoystickButton(operatorXbox, OperatorConstants.CLIMBER_DOWN).onTrue(new ClimberCmd(climberSubsystem, -1)); // reverse intake
    new JoystickButton(operatorXbox, OperatorConstants.INDEXER_FORWARD).onTrue(new IndexerCmd(indexerSubsystem, 1)); //intake
    new JoystickButton(operatorXbox, OperatorConstants.INDEXER_REVERSE).onTrue(new IndexerCmd(indexerSubsystem, -1)); // reverse intake
    new JoystickButton(operatorXbox, OperatorConstants.ELEVATOR_HIGH).onTrue(new ElevatorPIDCmd(elevatorSubsystem, OperatorPIDConstants.ELEVATOR_MAX_HEIGHT));
    new JoystickButton(operatorXbox, OperatorConstants.ELEVATOR_LOW).onTrue(new ElevatorPIDCmd(elevatorSubsystem, OperatorPIDConstants.ELEVATOR_MIN_HEIGHT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // START OF GETAUTONOMOUSCOMMAND
   public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
