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

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignTargetOdometry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;

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

        private final FeederSubsystem m_feeder = new FeederSubsystem(false);

        private final IntakeSubsystem m_intake = new IntakeSubsystem(true);

        private final IntakeArmSubsystem m_intakeArm = new IntakeArmSubsystem(true);

        public RobotContainer() {
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);
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

                // driver.leftTrigger().onTrue(m_shooter.spinUpCommand(1000));

                driver.rightTrigger().onTrue(m_intake.startIntakeCommand());

                driver.rightBumper().onTrue(
                                new SequentialCommandGroup(
                                                m_shooter.stopCommand(),
                                                m_feeder.stopFeederRollerCommand(),
                                                m_feeder.stopFeederBeltCommand(),
                                                m_intake.stopIntakeCommand()));

                driver.leftBumper().whileTrue(
                                new AlignTargetOdometry(drivetrain, m_shooter, drive, driver, false));

                driver.y().onTrue(Commands.none());

                driver.a().onTrue(Commands.none());

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

                codriver.y().onTrue(m_intakeArm.intakeArmToClearPositionCommand());

                codriver.a().onTrue(m_intakeArm.intakeArmToIntakePositionCommand());

                codriver.leftBumper().onTrue(m_shooter.stopAllShootersCommand());

                codriver.rightBumper().onTrue(
                                m_shooter.runVelocityVoltageCommand(m_shooter.leftMotor, 100));
                              
                codriver.y().onTrue(m_shooter.setDutyCycleCommand(m_shooter.leftMotor, .05))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.leftMotor, .0));
                codriver.a().onTrue(m_shooter.setVoltageCommand(m_shooter.leftMotor, .5))
                                .onFalse(m_shooter.setVoltageCommand(m_shooter.leftMotor, .0));

                // codriver.y().onTrue(m_shooter.changeFlywheelRPMCommand(100));
                // codriver.a().onTrue(m_shooter.changeFlywheelRPMCommand(-100));

                // codriver.y().onTrue(new InstantCommand(() -> flywheelPower += .05));
                // codriver.a().onTrue(new InstantCommand(() -> flywheelPower -= .05));

                // codriver.povUp().whileTrue(Commands.none());
                codriver.povDown().whileTrue(Commands.none());
                codriver.povLeft().whileTrue(m_intake.jogExtakeCommand());
                codriver.povRight().whileTrue(m_intake.jogIntakeCommand());

        }

        private void buildAutoChooser() {
                // Build an auto chooser. This will use Commands.none() as the default option.
                // As an example, this will only show autos that start with "comp" while at
                // competition as defined by the programmer

                autoChooser = AutoBuilder.buildAutoChooser("NewAuto");

                SmartDashboard.putData("PPAutoChooser", autoChooser);
        }

        private void registerNamedCommands() {

        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
                // return new PathPlannerAuto("SimpleAuto");
        }
}
