package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANIDConstants;

public class Intake4BarArmSubsystem extends SubsystemBase {

  private final SparkMax intakeArmMotor = new SparkMax(CANIDConstants.intakeArmID, MotorType.kBrushless);
  private final SparkMax intakeArmMotorFollower = new SparkMax(CANIDConstants.intakeArmFollowerID,
      MotorType.kBrushless);

  private static final double gearRatio = 57.38;

  private static final double radianspermotorev = (2 * Math.PI) / gearRatio;// approx .1

  private static double maxMotorRPS = 5700 / 60;// 95 approx

  private static double maxArmRadiansPerSec = maxMotorRPS * radianspermotorev;// approx 9

  public static double positionConversionFactor = radianspermotorev;
  public static double velocityConversionFactor = positionConversionFactor / 60; // radians per sec

  private static double kDt = 0.02;

  private static double kMaxTrapVelocity = Math.toRadians(250.);
  private static double kMaxTrapAcceleration = Math.toRadians(500);

  private DoubleSubscriber kp;
  private DoubleSubscriber ki;
  private DoubleSubscriber kd;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxTrapVelocity,
      kMaxTrapAcceleration);
  public ProfiledPIDController m_controller;
  private ArmFeedforward m_feedforward;

  public static Angle maxAngle = Degree.of(100);
  public static Angle minAngle = Degree.of(5);
  public static Angle zeroOffsetAngle = Degrees.of(65);
  public Angle intakingAngle = Degree.of(95);

  public Angle homeAngle = Degree.of(0);
  public Angle midUpAngle = Degree.of(30);
  public Angle midDownAngle = Degree.of(60);
  private Current stallCurrent = Amps.of(15);

  private Debouncer stallDebouncer = new Debouncer(.5);

  private double ks = .2;
  private double kg = 0.5;
  private double kv = 12 / maxArmRadiansPerSec;

  public boolean showData;
  private boolean alternate;

  public Intake4BarArmSubsystem(boolean showData) {

    intakeArmMotor
        .configure(
            Configs.IntakeArm.armConfig1,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

    intakeArmMotorFollower
        .configure(
            Configs.IntakeArm.armConfig2,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */

    this.showData = showData;

    m_feedforward = new ArmFeedforward(ks, kg, kv);

    intakeArmMotor.getEncoder().setPosition(homeAngle.in(Radians));

    kp = DogLog.tunable("IntakeArm/PGain", 8., newKp -> m_controller.setP(newKp));
    ki = DogLog.tunable("IntakeArm/IGain", .0, newKi -> m_controller.setI(newKi));
    kd = DogLog.tunable("IntakeArm/DGain", .0, newKd -> m_controller.setI(newKd));
    m_controller = new ProfiledPIDController(kp.get(), ki.get(), kd.get(), m_constraints, kDt);
    m_controller.setTolerance(Units.degreesToRadians(10));
    m_controller.setGoal(homeAngle.in(Radians));

    if (showData)
      SmartDashboard.putData(this);
  }

  public boolean getForwardSoftLimit(SparkMax motor) {
    return motor.getForwardSoftLimit().isReached();
  }

  public boolean getReverseSoftLimit(SparkMax motor) {
    return motor.getReverseSoftLimit().isReached();
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.setSmartDashboardType("IntakeArm");
    builder.addDoubleProperty("Motor Volts", () -> intakeArmMotor.getAppliedOutput() * 12, null);
    builder.addDoubleProperty("Motor Amps", () -> intakeArmMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("ActualPosition", () -> getIntakeArmAngle().in(Degrees), null);
    builder.addDoubleProperty("GoalPosition", () -> m_controller.getGoal().position, null);
    builder.addDoubleProperty("Velocity", () -> intakeArmMotor.getEncoder().getVelocity(), null);
    builder.addBooleanProperty("AtFwdSoftLimit", () -> getForwardSoftLimit(intakeArmMotor), null);
    builder.addBooleanProperty("AtRevSoftLimit", () -> getReverseSoftLimit(intakeArmMotor), null);

  }

  public void periodic() {
    if (alternate) {
      DogLog.log("IntakeArm/CurrentAngle", getIntakeArmAngle().in(Degrees));
      DogLog.log("IntakeArm/Encoder", intakeArmMotor.getEncoder().getPosition());
      DogLog.log("IntakeArm/EncoderVel", intakeArmMotor.getEncoder().getVelocity());
      DogLog.log("IntakeArm/MotorVolts", intakeArmMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
      DogLog.log("IntakeArm/FollowerEncoder", intakeArmMotorFollower.getEncoder().getPosition());
      DogLog.log("IntakeArm/FwdSoftLimit", intakeArmMotor.getForwardSoftLimit().isReached());
      DogLog.log("IntakeArm/RevSoftLimit", intakeArmMotor.getReverseSoftLimit().isReached());
    } else {

      DogLog.log("IntakeArm/TargetAngle", m_controller.getGoal().position);
      DogLog.log("IntakeArm/AngleError", m_controller.getPositionError());
      DogLog.log("IntakeArm/AtTarget", m_controller.atGoal());
      DogLog.log("IntakeArm/LeaderAmps", intakeArmMotor.getOutputCurrent());
      DogLog.log("IntakeArm/FollowerAmps", intakeArmMotorFollower.getOutputCurrent());
    }

    alternate = !alternate;
  }

  public void simulationPeriodic() {

  }

  public Command helpShootCommand(double pauseBetween) {

    return Commands.sequence(
        intakeArmToMidDownAngleCommand(),
        Commands.waitSeconds(pauseBetween),
        intakeArmToMidUpAngleCommand(),
        Commands.waitSeconds(pauseBetween))
        .repeatedly();
  }

  public boolean armInPosition() {
    return m_controller.atGoal();
    // return Math.abs(m_controller.getGoal().position -
    // intakeArmMotor.getEncoder().getPosition()) < .2;// 10 degrees
  }

  public Command intakeArmToIntakeAngleCommand() {
    return Commands.runOnce(() -> m_controller.setGoal(intakingAngle.in(Radians)));
  }

  public Command intakeArmToClearAngleCommand() {
    return Commands.runOnce(() -> m_controller.setGoal(homeAngle.in(Radians)));
  }

  public Command intakeArmToMidUpAngleCommand() {
    return Commands.runOnce(() -> m_controller.setGoal(midUpAngle.in(Radians)));
  }

  public Command intakeArmToMidDownAngleCommand() {
    return Commands.runOnce(() -> m_controller.setGoal(midDownAngle.in(Radians)));
  }

  public Command positionIntakeArmCommand() {
    return Commands.run(() -> positionIntakeArm(), this);
  }

  public void positionIntakeArm() {
    double ff = 0;// m_feedforward.calculate((getIntakeArmAngle().in(Radians) -
                  // zeroOffsetAngle.in(Radians)),
    // m_controller.getSetpoint().velocity);
    double pidout = m_controller.calculate(getIntakeArmAngle().in(Radians));
    intakeArmMotor.setVoltage(ff + pidout);
  }

  public Angle getIntakeArmAngle() {
    return Radians.of(intakeArmMotor.getEncoder().getPosition());
  }

  public Angle getIntakeFollowerAngle() {
    return Radians.of(intakeArmMotorFollower.getEncoder().getPosition());
  }

  public AngularVelocity getIntakeArmVelocity() {
    return DegreesPerSecond.of(intakeArmMotor.getEncoder().getVelocity());
  }

  public Current getMotorCurrent() {
    return Amps.of(intakeArmMotor.getOutputCurrent());
  }

  public boolean stalledAtEndTravel() {
    boolean stalled = getMotorCurrent().gt(stallCurrent);
    return stallDebouncer.calculate(stalled);
  }

  public Command jogIntakeArmCommand(DoubleSupplier jogRate) {
    return new FunctionalCommand(
        () -> {
        }, // init
        () -> {
          intakeArmMotor.setVoltage(jogRate.getAsDouble() * RobotController.getBatteryVoltage());
        }, // execute
        (interrupted) -> {
          intakeArmMotor.set(0);
          m_controller.setGoal(getIntakeArmAngle().in(Radians));
        }, // end
        () -> false, // isFinished
        this);// requirements
  }

}
