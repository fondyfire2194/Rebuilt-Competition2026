package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANIDConstants;

public class IntakeArmSubsystem extends SubsystemBase {

  private final SparkMax intakeArmMotor = new SparkMax(CANIDConstants.intakeArmID, MotorType.kBrushless);

  SparkMaxConfig armConfig;

  private static double gearRatio = 36;// 1 motor rev = 10 degrees

  private static double maxMotorRPS = 5700 / 60;

  private static double maxArmRadsPerSec = Units.degreesToRadians(maxMotorRPS * 360. / gearRatio);

  public static double positionConversionFactor = 2 * Math.PI / gearRatio;// rads per motor rev
  public static double velocityConversionFactor = positionConversionFactor / 60; // degrees per sec

  private static double kDt = 0.02;

  private static double kMaxTrapVelocity = Units.degreesToRadians(100.);
  private static double kMaxTrapAcceleration = Units.degreesToRadians(200.);

  private static double kP = .05;
  private static double kI = 0.0;
  private static double kD = 0.0;

  private static double kS = .1;
  private static double kG = 0.;
  private static double kV = 12. / maxArmRadsPerSec;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxTrapVelocity,
      kMaxTrapAcceleration);
  public final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV);

  public static Angle maxAngle = Degrees.of(100);
  public static Angle minAngle = Degrees.of(-10);

  public Angle intakingAngle = Degrees.of(55);

  public Angle clearAngle = Degrees.of(-20);

  private int tst;
  
  public boolean showData;

  public IntakeArmSubsystem(boolean showData) {

    m_controller.setGoal(minAngle.in(Radians));

    intakeArmMotor.configure(
        Configs.IntakeArm.intakeArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        this.showData = showData;
    if (showData)
      SmartDashboard.putData(this);

    intakeArmMotor.getEncoder().setPosition(minAngle.in(Radians));
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.setSmartDashboardType("IntakeArm");
    builder.addDoubleProperty("ActualAngle", () -> getIntakeArmAngle().in(Degrees), null);
    builder.addDoubleProperty("GoalAngle", () -> m_controller.getGoal().position, null);
    builder.addDoubleProperty("Velocity", () -> getIntakeArmVelocity().in(DegreesPerSecond), null);

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
    return Commands.runOnce(() -> m_controller.setGoal(intakingAngle.in(Radian)));
  }

  public Command intakeArmToClearPositionCommand() {
    return Commands.runOnce(() -> m_controller.setGoal(clearAngle.in(Radian)));
  }

  public Command positionIntakeArmCommand() {
    return Commands.run(() -> positionIntakeArm(), this);
  }

  public void positionIntakeArm() {
    tst++;
    SmartDashboard.putNumber("IATEST", tst);
    double feedforward = m_feedforward.calculate(m_controller.getSetpoint().position,
        m_controller.getSetpoint().velocity);
    double pidout = m_controller.calculate(getIntakeArmAngle().in(Radian));
    intakeArmMotor.setVoltage(feedforward + pidout);
  }

  public Angle getIntakeArmAngle() {
    return Degrees.of(intakeArmMotor.getEncoder().getPosition());
  }

  public AngularVelocity getIntakeArmVelocity() {
    return DegreesPerSecond.of(intakeArmMotor.getEncoder().getVelocity());
  }

  public Command jogIntakeArmCommand(DoubleSupplier speed) {
    return new FunctionalCommand(
        () -> {
        }, // init
        () -> {
          if (getIntakeArmAngle().lt(maxAngle) && speed.getAsDouble() > 0)
            intakeArmMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage());
          else if (getIntakeArmAngle().gte(minAngle) && speed.getAsDouble() < 0)
            intakeArmMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage());
          else
            intakeArmMotor.set(0);
        }, // execute
        (interrupted) -> intakeArmMotor.set(0), // end
        () -> false, // isFinished
        this);// requirements
  }

}
