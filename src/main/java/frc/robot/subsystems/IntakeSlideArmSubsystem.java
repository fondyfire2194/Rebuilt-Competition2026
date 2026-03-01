package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  private final SparkMax intakeArmSlideMotor = new SparkMax(CANIDConstants.intakeArmID, MotorType.kBrushless);

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

  private static double kP = .05;
  private static double kI = 0.0;
  private static double kD = 0.0;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxTrapVelocity,
      kMaxTrapAcceleration);
  public final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);

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

  private double jogInSpeed = -.15;

  private double jogOutSpeed = .15;

  public IntakeSlideArmSubsystem(boolean showData) {

    m_controller.setGoal(homeDistance.in(Inches));

    intakeArmSlideMotor.configure(
        Configs.IntakeSlideArm.intakeSlideArmConfig,
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

    intakeArmSlideMotor.configure(
        Configs.IntakeSlideArm.intakeSlideArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        
    this.showData = showData;

    if (showData)
      SmartDashboard.putData(this);

    slideFeedforward = new SimpleMotorFeedforward(ks, kv);

    intakeArmSlideMotor.getEncoder().setPosition(homeDistance.in(Inches));
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.setSmartDashboardType("IntakeArm");
    builder.addDoubleProperty("Motor Volts", () -> intakeArmSlideMotor.getAppliedOutput() * 12, null);
    builder.addDoubleProperty("ActualPosition", () -> getIntakeSlidePosition().in(Inches), null);
    builder.addDoubleProperty("GoalPosition", () -> m_controller.getGoal().position, null);
    builder.addDoubleProperty("Velocity", () -> getIntakeSlideVelocity().in(InchesPerSecond), null);
    builder.addBooleanProperty("AtSetpoint", m_controller::atSetpoint, null);
  }

  public void periodic() {

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
    intakeArmSlideMotor.setVoltage(ff = pidout);
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

  public Command jogIntakeArmCommand() {
    return new FunctionalCommand(
        () -> {
        }, // init
        () -> {
          if (getIntakeSlidePosition().lt(maxDistance) && jogOutSpeed > 0)
            intakeArmSlideMotor.setVoltage(jogOutSpeed * RobotController.getBatteryVoltage());
          else if (getIntakeSlidePosition().gte(minDistance) && jogInSpeed < 0)
            intakeArmSlideMotor.setVoltage(jogInSpeed * RobotController.getBatteryVoltage());
          else
            intakeArmSlideMotor.set(0);
        }, // execute
        (interrupted) -> intakeArmSlideMotor.set(0), // end
        () -> false, // isFinished
        this);// requirements
  }

}
