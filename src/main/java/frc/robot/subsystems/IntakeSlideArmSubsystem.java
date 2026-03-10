package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANIDConstants;
import frc.robot.utils.Logger;

public class IntakeSlideArmSubsystem extends SubsystemBase {

  private final SparkMax intakeArmSlideMotor = new SparkMax(CANIDConstants.intakeArmID, MotorType.kBrushless);
  private final SparkMax intakeArmSlideMotorFollower = new SparkMax(CANIDConstants.intakeArmFollowerID,
      MotorType.kBrushless);

  SimpleMotorFeedforward slideFeedforward;

  static double motorPulleyTeeth = 18;
  static double beltPulleyTeeth = 18;
  static Distance motorToFirstPulleyBeltPitch = Millimeters.of(215);
  static Distance firstPulleyToShaftBeltPitch = Millimeters.of(650);

  private static double inchesperMotorRev = (motorPulleyTeeth / beltPulleyTeeth)
      * ((motorToFirstPulleyBeltPitch.in(Inches) / firstPulleyToShaftBeltPitch.in(Inches)));// approx .33 inches

  private static double maxMotorRPS = 5700 / 60;// 95 approx

  private static double maxSlideInchesPerSec = maxMotorRPS * inchesperMotorRev;

  public static double positionConversionFactor = 1 / inchesperMotorRev;// inches per motor rev
  public static double velocityConversionFactor = positionConversionFactor / 60; // degrees per sec

  private static double kDt = 0.02;

  private static double kMaxTrapVelocity = 20;
  private static double kMaxTrapAcceleration = 40;

  private DoubleSubscriber kp;
  private DoubleSubscriber ki;
  private DoubleSubscriber kd;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxTrapVelocity,
      kMaxTrapAcceleration);
  public ProfiledPIDController m_controller;

  public static Distance maxDistance = Inches.of(10);
  public static Distance minDistance = Inches.of(-1);

  public Distance intakingDistance = Inches.of(8);

  public Distance homeDistance = Inches.of(0);

  private Current stallCurrent = Amps.of(15);

  private Debouncer stallDebouncer = new Debouncer(.5);

  private double ks = .1;
  private double kv = 12 / maxSlideInchesPerSec;

  private int tst;

  public boolean showData;

  private final Alert intakeArmAlert = new Alert(
      "Intake Arm Fault",
      AlertType.kError);

  public IntakeSlideArmSubsystem(boolean showData) {

    intakeArmSlideMotor
        .configure(
            Configs.IntakeSlideArm.configLeader,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

    intakeArmSlideMotorFollower
        .configure(
            Configs.IntakeSlideArm.configFollower,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

    Configs.IntakeSlideArm.configFollower.follow(CANIDConstants.intakeArmID);
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

    slideFeedforward = new SimpleMotorFeedforward(ks, kv);

    intakeArmSlideMotor.getEncoder().setPosition(homeDistance.in(Inches));

    kp = DogLog.tunable("IntakeSlideArm/PGain", .03, newKp -> m_controller.setP(newKp));
    ki = DogLog.tunable("IntakeSlideArm/IGain", .0, newKi -> m_controller.setI(newKi));
    kd = DogLog.tunable("IntakeSlideArm/DGain", .0, newKd -> m_controller.setI(newKd));
    m_controller = new ProfiledPIDController(kp.get(), ki.get(), kd.get(), m_constraints, kDt);

    m_controller.setGoal(homeDistance.in(Inches));

    if (showData)
      SmartDashboard.putData(this);

    intakeArmAlert.set(intakeArmSlideMotor.hasActiveFault() || intakeArmSlideMotor.hasStickyFault()
        || intakeArmSlideMotorFollower.hasActiveFault() || intakeArmSlideMotorFollower.hasStickyFault());
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.setSmartDashboardType("IntakeArm");

    builder.addDoubleProperty("Motor Volts", () -> intakeArmSlideMotor.getAppliedOutput() * 12, null);
    builder.addDoubleProperty("Motor Amps", () -> intakeArmSlideMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("ActualPosition", () -> getIntakeSlidePosition().in(Inches), null);
    builder.addDoubleProperty("GoalPosition", () -> m_controller.getGoal().position, null);
    builder.addDoubleProperty("Velocity", () -> getIntakeSlideVelocity().in(InchesPerSecond), null);
    builder.addBooleanProperty("AtSetpoint", m_controller::atSetpoint, null);

  }

  public void periodic() {
    Logger.log("IntakeSlideArm/Position", getIntakeSlidePosition());
    Logger.log("IntakeSlideArm/Velocity", getIntakeSlideVelocity());
    Logger.log("IntakeSlideArm/Volts", intakeArmSlideMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    Logger.log("IntakeSlideArm/Amps", intakeArmSlideMotor.getOutputCurrent());
    Logger.log("IntakeSlideArm/Goal", m_controller.getGoal().position);

  }

  public void simulationPeriodic() {

  }

  public boolean armSlideInPosition() {
    return Math.abs(m_controller.getGoal().position - intakeArmSlideMotor.getEncoder().getPosition()) < .25;
  }

  public Command intakeArmSlideToIntakePositionCommand() {
    return Commands.runOnce(() -> m_controller.setGoal(intakingDistance.in(Inches)));
  }

  public Command intakeArmSlideToClearPositionCommand() {
    return Commands.runOnce(() -> m_controller.setGoal(homeDistance.in(Inches)));
  }

  public Command positionIntakeArmSlideCommand() {
    return Commands.run(() -> positionIntakeSlide(), this);
  }

  public void positionIntakeSlide() {
    double ff = slideFeedforward.calculate(getIntakeSlidePosition().in(Inches));
    double pidout = m_controller.calculate(getIntakeSlidePosition().in(Inches));
    intakeArmSlideMotor.setVoltage(ff + pidout);
  }

  public Distance getIntakeSlidePosition() {
    return Inches.of(intakeArmSlideMotor.getEncoder().getPosition());
  }

  public LinearVelocity getIntakeSlideVelocity() {
    return InchesPerSecond.of(intakeArmSlideMotor.getEncoder().getVelocity());
  }

  public Current getMotorCurrent() {
    return Amps.of(intakeArmSlideMotor.getOutputCurrent());
  }

  public boolean stalledAtEndTravel() {
    boolean stalled = getMotorCurrent().gt(stallCurrent);
    return stallDebouncer.calculate(stalled);
  }

  public Command clearIntakeArmStickyFaultsCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> intakeArmSlideMotor.clearFaults()),
        Commands.runOnce(() -> intakeArmSlideMotorFollower.clearFaults()));
  }

  public Command jogIntakeArmCommand(DoubleSupplier jogRate) {
    return new FunctionalCommand(
        () -> {
        }, // init
        () -> {
          if (getIntakeSlidePosition().lt(maxDistance) && jogRate.getAsDouble() > 0)
            intakeArmSlideMotor.setVoltage(jogRate.getAsDouble() * RobotController.getBatteryVoltage());
          else if (getIntakeSlidePosition().gte(minDistance) && jogRate.getAsDouble() < 0)
            intakeArmSlideMotor.setVoltage(jogRate.getAsDouble() * RobotController.getBatteryVoltage());
          else
            intakeArmSlideMotor.set(0);
        }, // execute
        (interrupted) -> intakeArmSlideMotor.set(0), // end
        () -> false, // isFinished
        this);// requirements
  }

}
