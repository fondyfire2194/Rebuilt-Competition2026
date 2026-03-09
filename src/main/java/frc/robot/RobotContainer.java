// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AlignTargetOdometry;
import frc.robot.commands.AutoAlignHub;
import frc.robot.commands.DriveWithShootOnTheMove;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSlideArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.TripleShooterSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.FuelSim;
import frc.robot.utils.ShootingData;
import frc.robot.utils.SimRobotFuelSim;

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

        private final CommandXboxController driver = new CommandXboxController(0);
        public final CommandXboxController codriver = new CommandXboxController(1);
        public final CommandXboxController presetdriver = new CommandXboxController(2);
        public final CommandXboxController bumpdriver = new CommandXboxController(3);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final Telemetry logger = new Telemetry(RobotConstants.MaxSpeed, drivetrain);


        /* Path follower */
        private SendableChooser<Command> autoChooser;

        final TripleShooterSubsystem m_shooter;

        final HoodSubsystem m_hood;

        private final FeederSubsystem m_feeder;

        final IntakeSubsystem m_intake;

        private final IntakeSlideArmSubsystem m_intakeArm;

        public final LimelightVision m_llv;

        public final AddressableLEDSubsystem m_leds;

        // public final PowerDistribution pdh;

        private boolean showShooterData = false;
        private boolean showHoodData = true;
        private boolean showFeederData = false;
        private boolean showIntakeData = true;
        private boolean showIntakeArmData = false;
        private boolean showLLData = false;

        private Trigger driverFiveSecondWarningEndShootTrigger;
        private Trigger driverFiveSecondWarningEndPickupTrigger;
        private Trigger endGameWarningTrigger;

        private Trigger autoShootTrigger;

        // Presets
        public static final double hubPresetDistance = 0.96;
        public static final double towerPresetDistance = 2.5;
        public static final double trenchPresetDistance = 3.03;
        public static final double outpostPresetDistance = 4.84;

        static DoubleSupplier hdminhminA;
        static DoubleSupplier hdminshspd;
        static DoubleSupplier hdmaxmaxA;
        static DoubleSupplier hdmaxshspd;

        static DoubleSupplier hubhminA;
        static DoubleSupplier hubshspd;
        static DoubleSupplier twrminA;
        static DoubleSupplier twrshspd;
        static DoubleSupplier trenchminA;
        static DoubleSupplier trenchshspd;
        static DoubleSupplier outpostminA;
        static DoubleSupplier outpostshspd;

        public static record LaunchPreset(
                        DoubleSupplier hoodAngleDeg, DoubleSupplier flywheelSpeed) {
        }

        public static LaunchPreset hoodMinPreset;
        public static LaunchPreset hoodMaxPreset;

        public static LaunchPreset outpostPreset;
        public static LaunchPreset hubPreset;
        public static LaunchPreset towerPreset;
        public static LaunchPreset trenchPreset;

        public RobotContainer() {

                m_shooter = new TripleShooterSubsystem(showShooterData);
                m_hood = new HoodSubsystem(showHoodData);
                m_feeder = new FeederSubsystem(showFeederData);
                m_intake = new IntakeSubsystem(showIntakeData);
                m_intakeArm = new IntakeSlideArmSubsystem(showIntakeArmData);
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
                configurePresets();
                configureDriverBindings();
                configureCodriverBindings();
                configurePresetControllerBindings();
                configureBumpControllerBindings();

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

                m_intakeArm.setDefaultCommand(m_intakeArm.positionIntakeArmSlideCommand());

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
                                Commands.defer(
                                                () -> shootCommand(driver.a().getAsBoolean()),
                                                Set.of()));

                driver.rightTrigger().whileTrue(
                                Commands.parallel(
                                                m_intakeArm.intakeArmSlideToIntakePositionCommand(),
                                                m_intake.startIntakeCommand()))
                                .onFalse(
                                                Commands.sequence(
                                                                Commands.waitSeconds(5),
                                                                m_intakeArm.intakeArmSlideToClearPositionCommand(),
                                                                m_intake.stopIntakeCommand()));

                driver.leftBumper().onTrue(
                                Commands.sequence(
                                                m_shooter.setShootUsingDistanceCommand(true),
                                                m_hood.setHoodUsingDistanceCommand(true),
                                                m_shooter.runAllVelocityVoltageCommand()))
                                .whileTrue(new DriveWithShootOnTheMove(drivetrain, m_hood, m_shooter, drive,
                                                driver));
                // .whileTrue(new AlignTargetOdometry(drivetrain, m_shooter, m_hood, drive,
                // driver, 1));

                driver.rightBumper().onTrue(
                                Commands.parallel(
                                                m_shooter.setShootUsingDistanceCommand(false),
                                                m_shooter.stopAllShootersCommand(),
                                                m_feeder.stopFeederRollerCommand(),
                                                m_feeder.stopFeederBeltCommand(),
                                                m_intake.stopIntakeCommand()))
                                .whileTrue(Commands.defer(this::driveAtBumpAngle, Set.of(drivetrain)));

                driver.b().onTrue(m_hood.setManualTargetCommand(HoodSubsystem.kMinPosition.in(Degrees)));

                driver.y().onTrue(
                                new DeferredCommand(() -> m_hood.incrementHoodCommand(.5), Set.of()));

                driver.a().onTrue(
                                new DeferredCommand(() -> m_hood.incrementHoodCommand(-.5), Set.of()));

                driver.x().onTrue(m_hood.setManualTargetCommand(HoodSubsystem.kMaxPosition.in(Degrees)));

                driver.povUp().onTrue(m_shooter.changeFinalTargetVelocityCommand(100));

                driver.povDown().onTrue(m_shooter.changeFinalTargetVelocityCommand(-100));

                driver.povLeft().onTrue(
                                new DeferredCommand(() -> Commands.either(
                                                Commands.parallel(m_shooter.setShootUsingDistanceCommand(false),
                                                                m_hood.setHoodUsingDistanceCommand(false)),
                                                Commands.parallel(m_shooter.setShootUsingDistanceCommand(true),
                                                                m_hood.setHoodUsingDistanceCommand(false)),
                                                () -> m_shooter.isShootUsingDistance()), Set.of()));

                driver.povRight().onTrue(Commands.runOnce(() -> drivetrain.resetRotation(new Rotation2d()))
                                .ignoringDisable(true));
                driver.back().onTrue(Commands.runOnce(() -> drivetrain.resetRotation(new Rotation2d())));
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

                codriver.leftBumper().onTrue(m_shooter.stopAllShootersCommand());

                codriver.rightBumper().onTrue(Commands.none());
                codriver.rightTrigger().and(codriver.povUp()).whileTrue(m_feeder.jogFeederBeltCommand());

                codriver.rightTrigger().and(codriver.povDown()).onTrue(
                                m_feeder.runBeltsAndRollersCommand());

                codriver.rightTrigger().and(codriver.y().whileTrue(
                                m_shooter.setDutyCycleCommand(m_shooter.middleMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.middleMotor, .0)));

                codriver.rightTrigger().and(codriver.a().whileTrue(
                                m_shooter.setDutyCycleCommand(m_shooter.leftMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.leftMotor, .0)));

                codriver.rightTrigger().and(codriver.x().whileTrue(
                                m_shooter.setDutyCycleCommand(m_shooter.rightMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.rightMotor, .0)));

                codriver.rightTrigger().and(codriver.povLeft().whileTrue(m_hood.jogHoodUpCommand()));

                codriver.rightTrigger().and(codriver.povRight().whileTrue(m_hood.jogHoodDownCommand()));

                codriver.leftTrigger().and(codriver.povUp()
                                .whileTrue(m_intakeArm.jogIntakeArmCommand()));

                codriver.leftTrigger().and(codriver.povRight().whileTrue(m_intake.jogExtakeCommand()));

                codriver.leftTrigger().and(codriver.povLeft().whileTrue(m_intake.jogIntakeCommand()));

        }

        public void configurePresetControllerBindings() {

                presetdriver.leftBumper().onTrue(
                                Commands.parallel(
                                                m_shooter.setShootUsingDistanceCommand(false),
                                                m_hood.setHoodUsingDistanceCommand(false),
                                                m_shooter.setManualTargetVelocityCommand(
                                                                RPM.of(hdminshspd.getAsDouble())),
                                                m_hood.setManualTargetCommand(hdminhminA.getAsDouble())));

                presetdriver.leftTrigger().onTrue(
                                Commands.parallel(
                                                m_shooter.setShootUsingDistanceCommand(false),
                                                m_hood.setHoodUsingDistanceCommand(false),
                                                m_shooter.setManualTargetVelocityCommand(
                                                                RPM.of(hdmaxshspd.getAsDouble())),
                                                m_hood.setManualTargetCommand(hdmaxmaxA.getAsDouble())));

                presetdriver.a().onTrue(
                                Commands.parallel(
                                                m_shooter.setShootUsingDistanceCommand(false),
                                                m_hood.setHoodUsingDistanceCommand(false),
                                                m_shooter.setManualTargetVelocityCommand(
                                                                RPM.of(twrshspd.getAsDouble())),
                                                m_hood.setManualTargetCommand(twrminA.getAsDouble())));

                presetdriver.b().onTrue(
                                Commands.parallel(
                                                m_shooter.setShootUsingDistanceCommand(false),
                                                m_hood.setHoodUsingDistanceCommand(false),
                                                m_shooter.setManualTargetVelocityCommand(
                                                                RPM.of(trenchshspd.getAsDouble())),
                                                m_hood.setManualTargetCommand(trenchminA.getAsDouble())));

                presetdriver.x().onTrue(
                                Commands.parallel(
                                                m_shooter.setShootUsingDistanceCommand(false),
                                                m_hood.setHoodUsingDistanceCommand(false),
                                                m_shooter.setManualTargetVelocityCommand(
                                                                RPM.of(outpostshspd.getAsDouble())),
                                                m_hood.setManualTargetCommand(outpostminA.getAsDouble())));

                presetdriver.a().onTrue(
                                Commands.parallel(
                                                m_shooter.setShootUsingDistanceCommand(false),
                                                m_hood.setHoodUsingDistanceCommand(false),
                                                m_shooter.setManualTargetVelocityCommand(
                                                                RPM.of(twrshspd.getAsDouble())),
                                                m_hood.setManualTargetCommand(twrminA.getAsDouble())));

                presetdriver.y().onTrue(
                                Commands.parallel(
                                                m_shooter.setShootUsingDistanceCommand(false),
                                                m_hood.setHoodUsingDistanceCommand(false),
                                                m_shooter.setManualTargetVelocityCommand(
                                                                RPM.of(hubshspd.getAsDouble())),
                                                m_hood.setManualTargetCommand(hubhminA.getAsDouble())));

                presetdriver.povLeft().onTrue(
                                new DeferredCommand(() -> Commands.either(
                                                Commands.parallel(m_shooter.setShootUsingDistanceCommand(false),
                                                                m_hood.setHoodUsingDistanceCommand(false)),
                                                Commands.parallel(m_shooter.setShootUsingDistanceCommand(true),
                                                                m_hood.setHoodUsingDistanceCommand(true)),
                                                () -> m_shooter.isShootUsingDistance()), Set.of()));

        }

        public void configureBumpControllerBindings() {

                // bumpdriver.leftBumper().whileTrue(drivetrain.applyRequest(() ->
                // forwardStraightVelocity
                // .withVelocityX(1.5).withVelocityY(0.)));

                bumpdriver.leftTrigger().whileTrue(drivetrain.applyRequest(() -> forwardStraightVelocity
                                .withVelocityX(2.).withVelocityY(0)));

                // bumpdriver.rightBumper().whileTrue(drivetrain.applyRequest(() ->
                // forwardStraightVelocity
                // .withVelocityX(2.5).withVelocityY(0)));

                bumpdriver.rightTrigger().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(2.25)
                                                .withVelocityY(0)
                                                .withTargetDirection(Rotation2d.fromDegrees(45))));

                bumpdriver.a().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(2.5)
                                                .withVelocityY(0)
                                                .withTargetDirection(Rotation2d.fromDegrees(45))));

                bumpdriver.b().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(2.75)
                                                .withVelocityY(0)
                                                .withTargetDirection(Rotation2d.fromDegrees(45))));

                bumpdriver.x().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(3.)
                                                .withVelocityY(0)
                                                .withTargetDirection(Rotation2d.fromDegrees(45))));

                bumpdriver.a().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(3.25)
                                                .withVelocityY(0)
                                                .withTargetDirection(Rotation2d.fromDegrees(45))));

                bumpdriver.y().onTrue(
                                drivetrain.applyRequest(() -> forwardStraight
                                                .withVelocityX(0).withVelocityY(1)));

                bumpdriver.leftBumper().whileTrue(Commands.defer(this::driveAtBumpAngle, Set.of(drivetrain)));

                bumpdriver.rightBumper().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(-bumpdriver.getLeftY() * RobotConstants.MaxSpeed)
                                                .withVelocityY(-bumpdriver.getLeftX() * RobotConstants.MaxSpeed)
                                                .withTargetDirection(Rotation2d.fromDegrees(45))));

                bumpdriver.povUp().onTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(new Rotation2d(Math.PI / 2))));
                bumpdriver.povRight().onTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-Math.PI / 2))));
                bumpdriver.povDown().onTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(new Rotation2d(Math.PI))));

        }

        private void configureTriggers() {

                autoShootTrigger = new Trigger(
                                () -> m_shooter.hubIsActive
                                                && m_shooter.isShootOnTheMove
                                                && drivetrain.alignedToTarget
                                                && drivetrain.isAligning
                                                && m_hood.isPositionWithinTolerance()
                                                && m_shooter.allVelocityInTolerance());

                autoShootTrigger.onTrue(new ShootCommand(m_shooter, m_hood, m_feeder,
                                drivetrain, false));

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

        private void configurePresets() {
                hoodMinPreset = new LaunchPreset(
                                hdminhminA = DogLog.tunable(
                                                "LaunchCalculator/Presets/HoodMin/HoodAngle",
                                                HoodSubsystem.kMinPosition.in(Degrees)),
                                hdminshspd = DogLog.tunable(
                                                "LaunchCalculator/Presets/HoodMin/FlywheelSpeed", 50.));

                hoodMaxPreset = new LaunchPreset(
                                hdmaxmaxA = DogLog.tunable(
                                                "LaunchCalculator/Presets/HoodMax/HoodAngle",
                                                HoodSubsystem.kMaxPosition.in(Degrees)),
                                hdmaxshspd = DogLog.tunable(
                                                "LaunchCalculator/Presets/HoodMax/FlywheelSpeed", 50.));

                hubPreset = new LaunchPreset(
                                hubhminA = DogLog.tunable(
                                                "LaunchCalculator/Presets/Hub/HoodAngle",
                                                ShootingData.hoodAngleMap.get(hubPresetDistance).getDegrees()),
                                hubshspd = DogLog.tunable(
                                                "LaunchCalculator/Presets/Hub/FlywheelSpeed",
                                                ShootingData.shooterSpeedMap.get(hubPresetDistance)));

                towerPreset = new LaunchPreset(
                                twrminA = DogLog.tunable(
                                                "LaunchCalculator/Presets/Tower/HoodAngle",
                                                ShootingData.hoodAngleMap.get(towerPresetDistance).getDegrees()),
                                twrshspd = DogLog.tunable(
                                                "LaunchCalculator/Presets/Tower/FlywheelSpeed",
                                                ShootingData.shooterSpeedMap.get(towerPresetDistance)));

                trenchPreset = new LaunchPreset(
                                trenchminA = DogLog.tunable(
                                                "LaunchCalculator/Presets/Trench/HoodAngle",
                                                ShootingData.hoodAngleMap.get(trenchPresetDistance).getDegrees()),
                                trenchshspd = DogLog.tunable(
                                                "LaunchCalculator/Presets/Trench/FlywheelSpeed",
                                                ShootingData.shooterSpeedMap.get(trenchPresetDistance)));

                outpostPreset = new LaunchPreset(
                                outpostminA = DogLog.tunable(
                                                "LaunchCalculator/Presets/Outpost/HoodAngle",
                                                ShootingData.hoodAngleMap.get(outpostPresetDistance).getDegrees()),
                                outpostshspd = DogLog.tunable(
                                                "LaunchCalculator/Presets/Outpost/FlywheelSpeed",
                                                ShootingData.shooterSpeedMap.get(outpostPresetDistance)));

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

                NamedCommands.registerCommand("ALIGN_AND_SHOOT",
                                Commands.deadline(
                                                Commands.waitSeconds(5),
                                                new AutoAlignHub(drivetrain, m_shooter, m_hood, 1),
                                                new ShootCommand(m_shooter, m_hood, m_feeder, drivetrain, false))
                                                .andThen(m_shooter.stopAllShootersCommand()));

                NamedCommands.registerCommand("SHOOT_COMMAND",
                                new ShootCommand(m_shooter, m_hood, m_feeder, drivetrain, false));
                NamedCommands.registerCommand("END_SHOOT_COMMAND", m_shooter.stopAllShootersCommand());

                NamedCommands.registerCommand("START_SHOOTERS", m_shooter.runAllVelocityVoltageCommand());
                NamedCommands.registerCommand("STOP_SHOOTERS", m_shooter.stopAllShootersCommand());

        }

        private void registerEventTriggers() {

                EventTrigger runIntake = new EventTrigger("RUN_INTAKE");
                runIntake.onTrue(
                                Commands.sequence(
                                                m_intake.startIntakeCommand(),
                                                m_intakeArm.intakeArmSlideToIntakePositionCommand()));
                EventTrigger endIntake = new EventTrigger("END_INTAKE");
                endIntake.onTrue(
                                Commands.sequence(
                                                m_intake.stopIntakeCommand(),
                                                m_intakeArm.intakeArmSlideToClearPositionCommand()));

        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
                // return new PathPlannerAuto("SimpleAuto");
        }

        private Command driveAtBumpAngle() {
                return Commands.sequence(
                                Commands.runOnce(() -> AllianceUtil.getBumpCrossAngle(drivetrain.getState().Pose)),
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                                                .withVelocityX(-driver.getLeftY() * RobotConstants.MaxSpeed)
                                                .withVelocityY(-driver.getLeftX() * RobotConstants.MaxSpeed)
                                                .withTargetDirection(AllianceUtil.bumpRotation2d)));

        }

        private Command shootCommand(boolean bypass) {
                return Commands.either(
                                Commands.run(() -> Robot.fuelRobotSim.launchFuel()),
                                new ShootCommand(m_shooter, m_hood, m_feeder, drivetrain, bypass),
                                () -> RobotBase.isSimulation());
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
