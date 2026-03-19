// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import java.util.Set;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AlignTargetOdometry;
import frc.robot.commands.AutoAlignHub;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.Intake4BarArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.TripleShooterSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ShootingData;

public class RobotContainer {

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(RobotConstants.MaxSpeed * 0.1)
                        .withRotationalDeadband(RobotConstants.MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.FieldCentric forwardStraightVelocity = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.Velocity);

        private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
                        .withDeadband(RobotConstants.MaxSpeed * 0.1)
                        .withRotationalDeadband(RobotConstants.MaxAngularRate * 0.1) // Add a 10% deadband
                        .withHeadingPID(7, 0, 0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control
                                                                                              // for drive motors

        public final CommandXboxController driver = new CommandXboxController(0);
        public final CommandXboxController codriver = new CommandXboxController(1);
        public final CommandXboxController presetdriver = new CommandXboxController(2);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final Telemetry logger = new Telemetry(RobotConstants.MaxSpeed, drivetrain);

        /* Path follower */
        private SendableChooser<Command> autoChooser;

        final TripleShooterSubsystem m_shooter;

        final HoodSubsystem m_hood;

        private final FeederSubsystem m_feeder;

        private final IntakeSubsystem m_intake;

        private final Intake4BarArmSubsystem m_intakeArm;

        public final LimelightVision m_llv;

        public final AddressableLEDSubsystem m_leds;

        // public final PowerDistribution pdh;

        private boolean showShooterData = false;
        private boolean showHoodData = false;
        private boolean showFeederData = false;
        private boolean showIntakeData = false;
        private boolean showIntakeArmData = false;

        private boolean showLLData = false;

        private Trigger driverFiveSecondWarningEndShootTrigger;
        private Trigger driverFiveSecondWarningEndPickupTrigger;
        private Trigger endGameWarningTrigger;

        private Trigger collisionTrigger;

        // Presets
        // public static final double hubPresetDistance = 0.96;
        public static final double towerPresetDistance = 4.41;
        public static final double trenchPresetDistance = 3.3;
        // public static final double outpostPresetDistance = 4.84;

        public RobotContainer() {

                m_shooter = new TripleShooterSubsystem(showShooterData);
                m_hood = new HoodSubsystem(showHoodData);
                m_feeder = new FeederSubsystem(showFeederData);
                m_intake = new IntakeSubsystem(showIntakeData);
                m_intakeArm = new Intake4BarArmSubsystem(showIntakeArmData);
                m_llv = new LimelightVision(showLLData);
                m_leds = new AddressableLEDSubsystem();
                // pdh = new PowerDistribution(CANIDConstants.pdh, ModuleType.kRev);
                registerNamedCommands();
                registerEventTriggers();
                // configurePDH();

                m_shooter.leftMotorActive = true;
                m_shooter.middleMotorActive = true;
                m_shooter.rightMotorActive = true;

                setDefaultCommands();
                configureDriverBindings();
                configureCodriverBindings();

                configureTriggers();
                buildAutoChooser();

                SignalLogger.setPath("media/sda1/ctre-logs");

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void setDefaultCommands() {
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                                                .withVelocityX(-driver.getLeftY() * RobotConstants.MaxSpeed)
                                                .withVelocityY(-driver.getLeftX() * RobotConstants.MaxSpeed)
                                                // negative X (left)
                                                .withRotationalRate(
                                                                -driver.getRightX() * RobotConstants.MaxAngularRate)));

                m_intakeArm.setDefaultCommand(m_intakeArm.positionIntakeArmCommand());

                m_hood.setDefaultCommand(m_hood.positionHoodCommand());
        }

        private void configureDriverBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
                // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // driver.b().whileTrue(drivetrain
                // .applyRequest(() -> point.withModuleDirection(
                // new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

                driver.leftTrigger().whileTrue(
                                Commands.parallel(
                                                new ShootCommand(m_shooter, m_hood, m_feeder, drivetrain, false),
                                                Commands.sequence(Commands.waitSeconds(5),
                                                                m_intakeArm.helpShootCommand(1))));

                driver.rightTrigger().whileTrue(
                                Commands.parallel(
                                                m_intakeArm.intakeArmToIntakeAngleCommand(),
                                                m_intake.startIntakeCommand()))
                                .onFalse(
                                                m_intake.stopIntakeCommand());

                driver.leftBumper().onTrue(
                                Commands.sequence(
                                                setForAutoShootValues(),
                                                m_shooter.runAllVelocityVoltageCommand()))
                                // .whileTrue(new DriveWithShootOnTheMove(drivetrain, m_hood, m_shooter, drive,
                                // driver));
                                .whileTrue(new AlignTargetOdometry(drivetrain, m_shooter, m_hood, drive,
                                                driver, 1));

                driver.rightBumper().onTrue(
                                Commands.parallel(
                                                setForAutoShootValues(),
                                                stopShootersFeedersIntake(),
                                                m_intakeArm.intakeArmToClearAngleCommand()));

                driver.y().onTrue(m_hood.setManualTargetCommand(HoodSubsystem.kMinPosition.in(Degrees)));

                driver.b().whileTrue(presetShoot(trenchPresetDistance));

                driver.x().whileTrue(presetShoot(towerPresetDistance));

                driver.a().onTrue(m_hood.setManualTargetCommand(HoodSubsystem.kMaxPosition.in(Degrees)));

                driver.povUp().onTrue(m_shooter.changeFinalTargetVelocityCommand(100));

                driver.povDown().onTrue(m_shooter.changeFinalTargetVelocityCommand(-100));

                driver.povLeft().onTrue(
                                new DeferredCommand(() -> Commands.either(
                                                Commands.parallel(m_shooter.setShootUsingDistanceCommand(false),
                                                                m_hood.setHoodUsingDistanceCommand(false)),
                                                Commands.parallel(m_shooter.setShootUsingDistanceCommand(true),
                                                                m_hood.setHoodUsingDistanceCommand(false)),
                                                () -> m_shooter.isShootUsingDistance()), Set.of()));

                driver.povRight().onTrue(Commands.none());

                driver.back().onTrue(Commands.runOnce(() -> drivetrain.getPigeon2().reset()));

                // Reset the field-centric heading
                driver.start().onTrue(
                                drivetrain.runOnce(drivetrain::seedFieldCentric));

        }

        private void configureCodriverBindings() {
                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                codriver.back().and(codriver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                codriver.back().and(codriver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

                codriver.start().and(codriver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                codriver.start().and(codriver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
                codriver.start().and(codriver.povRight()).onTrue(Commands.runOnce(() -> SignalLogger.stop()));

                codriver.leftBumper()
                                .whileTrue(m_intakeArm.jogIntakeArmCommand(() -> codriver.getLeftX() / 4));

                codriver.rightBumper().and(codriver.y()).onTrue(m_intakeArm.intakeArmToClearAngleCommand());

                codriver.rightBumper().and(codriver.a()).onTrue(m_intakeArm.intakeArmToIntakeAngleCommand());

                codriver.rightBumper().and(codriver.b()).onTrue(m_intakeArm.intakeArmToMidUpAngleCommand());

                codriver.rightBumper().and(codriver.x()).onTrue(m_intakeArm.intakeArmToMidDownAngleCommand());

                codriver.rightBumper().and(codriver.povUp()).whileTrue(m_intakeArm.helpShootCommand(1));

                codriver.rightTrigger().and(codriver.povUp()).whileTrue(m_feeder.jogFeederBeltCommand());

                codriver.rightTrigger().and(codriver.povDown()).onTrue(
                                m_feeder.runBeltsAndRollersCommand());

                codriver.rightTrigger().and(codriver.y()).whileTrue(
                                m_shooter.setDutyCycleCommand(m_shooter.middleMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.middleMotor, .0));

                codriver.rightTrigger().and(codriver.a()).whileTrue(
                                m_shooter.setDutyCycleCommand(m_shooter.leftMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.leftMotor, .0));

                codriver.rightTrigger().and(codriver.x()).whileTrue(
                                m_shooter.setDutyCycleCommand(m_shooter.rightMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.rightMotor, .0));

                codriver.rightTrigger().and(codriver.povLeft()).whileTrue(m_hood.jogHoodUpCommand());

                codriver.rightTrigger().and(codriver.povRight()).whileTrue(m_hood.jogHoodDownCommand());

                codriver.leftTrigger().and(codriver.povLeft())
                                .onTrue(m_hood.setHoodZeroCommand().ignoringDisable(true));

                codriver.leftTrigger().and(codriver.povUp())
                                .onTrue(Commands.sequence(
                                                Commands.runOnce(() -> m_llv.useMT1 = true),
                                                (Commands.runOnce(() -> m_llv.useMT2 = false))));

                codriver.leftTrigger().and(codriver.povDown()).whileTrue(m_intake.jogIntakeCommand());

        }

        private void configureTriggers() {

                collisionTrigger = new Trigger(() -> drivetrain.jerkLimitExceeded);

                // collisionTrigger.onTrue(m_intakeArm.intakeArmUpCommand());

                driverFiveSecondWarningEndPickupTrigger = new Trigger(() -> m_leds.fiveSecondWarningEndOfPickup);

                driverFiveSecondWarningEndPickupTrigger
                                .onTrue(Commands.sequence(
                                                Commands.runOnce(() -> driver
                                                                .setRumble(RumbleType.kLeftRumble, 1)),
                                                Commands.waitSeconds(.5),
                                                Commands.runOnce(() -> driver
                                                                .setRumble(RumbleType.kLeftRumble, 0))));

                driverFiveSecondWarningEndShootTrigger = new Trigger(() -> m_leds.fiveSecondWarningEndOfShoot);
                driverFiveSecondWarningEndShootTrigger
                                .onTrue(Commands.sequence(
                                                Commands.runOnce(() -> driver.setRumble(RumbleType.kRightRumble, 1)),
                                                Commands.waitSeconds(.5),
                                                Commands.runOnce(() -> driver.setRumble(RumbleType.kRightRumble, 0))));

                endGameWarningTrigger = new Trigger(() -> m_leds.endGameWarning);
                endGameWarningTrigger
                                .onTrue(
                                                Commands.sequence(
                                                                Commands.runOnce(() -> driver
                                                                                .setRumble(RumbleType.kBothRumble, 1)),
                                                                Commands.waitSeconds(.75),
                                                                Commands.runOnce(() -> driver.setRumble(
                                                                                RumbleType.kBothRumble, 0))));

        }

        private void configurePDH() {

                /**
                 * 0 - BRSteer
                 * 1 -
                 * 2-
                 * 3-
                 * codriver.povLeft().onTrue(m_hood.positionToHomeCommand());
                 * 
                 * codriver.povRight().whileTrue(m_hood.positionTestCommand());
                 * 
                 * 4 -
                 * 5- intake slide arm
                 * 6 - FRSteer
                 * 7
                 * 8
                 * 9 - FRDrive
                 * 10 - FLDrive
                 * 11-
                 * 12-
                 * 13- FLSteer
                 * 14-
                 * 15-
                 * 16
                 * 17-BLSteer
                 * 18-BRDrive
                 * 19-BLDrive
                 * 
                 */

        }

        private void buildAutoChooser() {
                // Build an auto chooser. This will use Commands.none() as the default option.
                // As an example, this will only show autos that start with "comp" while at
                // competition as defined by the programmer

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("PPAutoChooser", autoChooser);
        }

        private void registerNamedCommands() {

                NamedCommands.registerCommand("START_SHOOTERS", m_shooter.runAllVelocityVoltageCommand());

                NamedCommands.registerCommand("ALIGN_AND_SHOOT",
                                Commands.deadline(
                                                Commands.waitSeconds(5),
                                                new AutoAlignHub(drivetrain, m_shooter, m_hood, 1),
                                                Commands.parallel(
                                                                new ShootCommand(m_shooter, m_hood, m_feeder,
                                                                                drivetrain, false),
                                                                Commands.sequence(
                                                                                Commands.waitSeconds(5),
                                                                                m_intakeArm.helpShootCommand(1))))
                                                .andThen(stopShootersFeedersIntake()));

        }

        private void registerEventTriggers() {

                EventTrigger startShooters = new EventTrigger("START_SHOOTERS");
                startShooters.onTrue(m_shooter.runAllVelocityVoltageCommand());

                EventTrigger runIntake = new EventTrigger("RUN_INTAKE");
                runIntake.onTrue(
                                Commands.sequence(
                                                m_intake.startIntakeCommand(),
                                                m_intakeArm.intakeArmToIntakeAngleCommand()));

                EventTrigger endIntake = new EventTrigger("END_INTAKE");
                endIntake.onTrue(m_intake.stopIntakeCommand());

        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
                // return new PathPlannerAuto("SimpleAuto");
        }

        public Command clearRevStickyFaultsCommand() {
                return Commands.sequence(
                                m_feeder.clearFeederStickyFaultsCommand(),
                                m_hood.clearHoodStickyFaultsCommand(),
                                m_intake.clearIntakeStickyFaultsCommand());
                // m_intakeSideArm.clearStickyFaultsCommand());
        }

        public Command presetShoot(double distance) {
                return Commands.sequence(
                                Commands.parallel(
                                                setForManualShootValues(),
                                                m_shooter.setManualTargetVelocityCommand(
                                                                RPM.of(ShootingData.shooterSpeedMap.get(distance))),
                                                m_hood.setManualTargetCommand(
                                                                ShootingData.hoodAngleMap.get(distance).getDegrees())),
                                m_shooter.runAllVelocityVoltageCommand(),
                                Commands.waitUntil(
                                                () -> m_shooter.allVelocityInTolerance()
                                                                && m_hood.isPositionWithinTolerance())
                                                .andThen(
                                                                Commands.parallel(
                                                                                new ShootCommand(m_shooter, m_hood,
                                                                                                m_feeder, drivetrain,
                                                                                                false),
                                                                                Commands.sequence(
                                                                                                Commands.waitSeconds(5),
                                                                                                m_intakeArm.helpShootCommand(
                                                                                                                1)))))
                                .finallyDo((() -> stopShootersFeedersIntake()));

        }

        public Command setForManualShootValues() {
                return Commands.parallel(
                                m_shooter.setShootUsingDistanceCommand(false),
                                m_hood.setHoodUsingDistanceCommand(false));

        }

        public Command setForAutoShootValues() {
                return Commands.parallel(
                                m_shooter.setShootUsingDistanceCommand(true),
                                m_hood.setHoodUsingDistanceCommand(true));

        }

        public Command setForPresetShootValues() {
                return Commands.parallel(
                                m_shooter.setShootUsingDistanceCommand(false),
                                m_hood.setHoodUsingDistanceCommand(false));

        }

        public Command stopShootersFeedersIntake() {
                return Commands.sequence(
                                m_shooter.stopAllShootersCommand(),
                                m_feeder.stopFeederRollerCommand(),
                                m_feeder.stopFeederBeltCommand(),
                                m_intake.stopIntakeCommand());

        }

        //
        // //Breakover Angle (B°) = 2 × tan-1(2 × Ground Clearance (GC) / Wheelbase
        // (WB)).
        // tan(B/2)/2 = Ground Clearance (GC) / Wheelbase (WB)
        // Ground Clearance = WheelBase * tan(B/2)/2
        // For B = 5 degrees tan/2 = .06
        // 20" WB GC = 1.2
        // 23" WB GC = 1.38"
        // 31" WB GC 1.8"
        // Track width is 2 * 11.6875 and wheel base is 2 * 10.1875 inches
        // Diagonal between wheels = 2*15.5 = 31
        // Ramp angle is 15 degrees and width is 44 inches
        // Ramp height is then 22 * tan 15 = 22 * .268 = 5.9 inches
        // Ramp slope length is 22/cos 15 = 22.77 inches

}
