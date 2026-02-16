// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlignHub;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.TripleShooterSubsystem;
import frc.robot.utils.CaptureMT1Values;
import frc.robot.utils.PickAndSetPosetoMT1;

public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driver = new CommandXboxController(0);
        public final CommandXboxController codriver = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        /* Path follower */
        private SendableChooser<Command> autoChooser;

        final TripleShooterSubsystem m_shooter = new TripleShooterSubsystem(true);

        final HoodSubsystem m_hood = new HoodSubsystem(true);

        private final FeederSubsystem m_feeder = new FeederSubsystem(true);

        private final IntakeSubsystem m_intake = new IntakeSubsystem(true);

        private final IntakeArmSubsystem m_intakeArm = new IntakeArmSubsystem(true);

        public final LimelightVision m_llv = new LimelightVision(true);

        public RobotContainer() {

                m_shooter.leftMotorActive = true;
                m_shooter.middleMotorActive = true;
                m_shooter.rightMotorActive = false;

                setDefaultCommands();
                configureDriverBindings();
                configureCodriverBindings();

                buildAutoChooser();

                registerNamedCommands();
                SignalLogger.setPath("media/sda1/ctre-logs");
                DogLog.setPdh(new PowerDistribution());
                DogLog.setPdh(null);

        }

        private void setDefaultCommands() {
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                                                .withVelocityX(-driver.getLeftY() * MaxSpeed)
                                                .withVelocityY(-driver.getLeftX() * MaxSpeed)
                                                // negative X (left)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

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
                // driver.a().whileTrue(m_feeder.feedCommand());

                // driver.povUp().whileTrue(
                // drivetrain.applyRequest(() -> forwardStraight
                // .withVelocityX(0.5).withVelocityY(0)));
                // driver.povDown().whileTrue(
                // drivetrain.applyRequest(() ->
                // forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

                driver.leftTrigger().whileTrue(new ShootCommand(m_shooter, m_hood, m_feeder));

                driver.rightTrigger().whileTrue(
                                Commands.parallel(
                                                m_intakeArm.intakeArmToIntakePositionCommand(),
                                                m_intake.startIntakeCommand()))
                                .onFalse(
                                                Commands.sequence(
                                                                Commands.waitSeconds(5),
                                                                m_intakeArm.intakeArmToClearPositionCommand(),
                                                                m_intake.stopIntakeCommand()));

                driver.rightBumper().onTrue(
                                Commands.parallel(
                                                m_shooter.stopAllShootersCommand(),
                                                m_feeder.stopFeederRollerCommand(),
                                                m_feeder.stopFeederBeltCommand(),
                                                m_intake.stopIntakeCommand()));

                // driver.leftBumper().whileTrue(
                // Commands.parallel(
                // new PrepareShotCommand(m_shooter, m_hood),
                // new AlignTargetOdometry(drivetrain, m_shooter, drive, driver, false)));
                driver.leftBumper().onTrue(m_shooter.runAllVelocityVoltageCommand());

                driver.y().whileTrue(m_intake.jogIntakeCommand());

                driver.a().whileTrue(m_intakeArm.jogIntakeArmCommand(() -> driver.getLeftX()));

                driver.b().onTrue(Commands.none());
                // Reset the field-centric heading on left bumper press.
                driver.start().onTrue(
                                drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);
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

                codriver.rightBumper().onTrue(
                                m_shooter.runAllVelocityVoltageCommand());

                codriver.y().onTrue(m_shooter.setDutyCycleCommand(m_shooter.middleMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.middleMotor, .0));

                codriver.a().onTrue(m_shooter.setVoltageCommand(m_shooter.middleMotor, 5))
                                .onFalse(m_shooter.setVoltageCommand(m_shooter.middleMotor, .0));

                codriver.povUp().onTrue(m_shooter.changeTargetVelocityCommand(100));

                codriver.povDown().onTrue(m_shooter.changeTargetVelocityCommand(-100));

                codriver.povLeft().onTrue(m_hood.positionToHomeCommand());

                codriver.povRight().whileTrue(m_hood.positionTestCommand());

                codriver.b().whileTrue(m_hood.jogHoodUpCommand());
                codriver.x().whileTrue(m_hood.jogHoodDownCommand());

                codriver.back().onTrue(
                                m_hood.positionToHomeCommand());

        }

        private void buildAutoChooser() {
                // Build an auto chooser. This will use Commands.none() as the default option.
                // As an example, this will only show autos that start with "comp" while at
                // competition as defined by the programmer

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("PPAutoChooser", autoChooser);
        }

        private void registerNamedCommands() {
                NamedCommands.registerCommand("USE_MT1",
                                Commands.sequence(
                                                new CaptureMT1Values(m_llv),
                                                new PickAndSetPosetoMT1(m_llv, drivetrain)));

                NamedCommands.registerCommand("USE_MT2", Commands.runOnce(() -> m_llv.useMT2 = true));

                NamedCommands.registerCommand("MOVE_TO_DEPOT_INTAKE_POSE",
                                drivetrain.pathFindToPose(new Pose2d(), drivetrain.pathConstraints));

                NamedCommands.registerCommand("START_INTAKE", m_intake.startIntakeCommand());
                NamedCommands.registerCommand("STOP_INTAKE", m_intake.stopIntakeCommand());

                NamedCommands.registerCommand("INTAKE_ARM_DOWN", m_intakeArm.intakeArmToIntakePositionCommand());
                NamedCommands.registerCommand("INTAKE_ARM_UP", m_intakeArm.intakeArmToClearPositionCommand());

                NamedCommands.registerCommand("ALIGN_TO_HUB", new AutoAlignHub(drivetrain, m_shooter, 1));

                NamedCommands.registerCommand("SHOOT_COMMAND", new ShootCommand(m_shooter, m_hood, m_feeder));
                NamedCommands.registerCommand("END_SHOOT_COMMAND", m_shooter.stopAllShootersCommand());

                NamedCommands.registerCommand("START_SHOOTERS", m_shooter.runAllVelocityVoltageCommand());
                NamedCommands.registerCommand("STOP_SHOOTERS", m_shooter.stopAllShootersCommand());

                NamedCommands.registerCommand("START CLIMBER", Commands.none());
                NamedCommands.registerCommand("STOP_CLIMBER", Commands.none());

        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
                // return new PathPlannerAuto("SimpleAuto");
        }
}
