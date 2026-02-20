package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANIDConstants;

public class IntakeSlideArmSubsystem extends SubsystemBase {

  private final SparkMax intakeArmMotor = new SparkMax(CANIDConstants.intakeArmID, MotorType.kBrushless);

  SparkMaxConfig armConfig;

  SimpleMotorFeedforward slideFeedforward;

  private static double gearRatio = 36;// 1 motor rev = 10 inches

  private static double maxMotorRPS = 5700 / 60;// 95 approx

  private static double maxSlideInchesPerSec = maxMotorRPS / gearRatio;

  public static double positionConversionFactor = 1 / gearRatio;// inches per motor rev
  public static double velocityConversionFactor = positionConversionFactor / 60; // degrees per sec

  private static double kDt = 0.02;

  private static double kMaxTrapVelocity = Units.degreesToRadians(100.);
  private static double kMaxTrapAcceleration = Units.degreesToRadians(200.);

  private static double kP = .05;
  private static double kI = 0.0;
  private static double kD = 0.0;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxTrapVelocity,
      kMaxTrapAcceleration);
  public final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);

  public static Distance maxDistance = Inches.of(100);
  public static Distance minDistance = Inches.of(0);

  public Distance intakingDistance = Inches.of(55);

  public Distance homeDistance = Inches.of(5);

  private Current stallCurrent = Amps.of(15);

  private Debouncer stallDebouncer = new Debouncer(.5);

  private double ks = .1;
  private double kv = 12 / maxSlideInchesPerSec;

  private int tst;

  public boolean showData;

  public IntakeSlideArmSubsystem(boolean showData) {

    m_controller.setGoal(homeDistance.in(Inches));

    intakeArmMotor.configure(
        Configs.IntakeArm.intakeSlideConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.showData = showData;
    if (showData)
      SmartDashboard.putData(this);

    slideFeedforward = new SimpleMotorFeedforward(ks, kv);

    intakeArmMotor.getEncoder().setPosition(homeDistance.in(Inches));
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.setSmartDashboardType("IntakeArm");
    builder.addDoubleProperty("ActualPosition", () -> getIntakeSlidePosition().in(Inches), null);
    builder.addDoubleProperty("GoalPosition", () -> m_controller.getGoal().position, null);
    builder.addDoubleProperty("Velocity", () -> getIntakeSlideVelocity().in(InchesPerSecond), null);
    builder.addBooleanProperty("AtSetpoint", m_controller::atSetpoint, null);
  }

  public void periodic() {

  }

  public void simulationPeriodic() {

  }

  public boolean armInPosition() {
    return Math.abs(m_controller.getGoal().position - intakeArmMotor.getEncoder().getPosition()) < .25;
  }

  public Command intakeArmToIntakePositionCommand() {
    return Commands.runOnce(() -> m_controller.setGoal(intakingDistance.in(Inches)));
  }

  public Command intakeArmToClearPositionCommand() {
    return Commands.runOnce(() -> m_controller.setGoal(homeDistance.in(Inches)));
  }

  public Command positionIntakeArmCommand() {
    return Commands.run(() -> positionIntakeSlide(), this);
  }

  public void positionIntakeSlide() {
    tst++;

  }

  public Distance getIntakeSlidePosition() {
    return Inches.of(intakeArmMotor.getEncoder().getPosition());
  }

  public LinearVelocity getIntakeSlideVelocity() {
    return InchesPerSecond.of(intakeArmMotor.getEncoder().getVelocity());
  }

  public Current getMotorCurrent() {
    return Amps.of(intakeArmMotor.getOutputCurrent());
  }

  public boolean stalledAtEndTravel() {
    boolean stalled = getMotorCurrent().gt(stallCurrent);
    return stallDebouncer.calculate(stalled);
  }

  public Command jogIntakeArmCommand(DoubleSupplier speed) {
    return new FunctionalCommand(
        () -> {
        }, // init
        () -> {
          if (getIntakeSlidePosition().lt(maxDistance) && speed.getAsDouble() > 0)
            intakeArmMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage());
          else if (getIntakeSlidePosition().gte(minDistance) && speed.getAsDouble() < 0)
            intakeArmMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage());
          else
            intakeArmMotor.set(0);
        }, // execute
        (interrupted) -> intakeArmMotor.set(0), // end
        () -> false, // isFinished
        this);// requirements
  }

}
