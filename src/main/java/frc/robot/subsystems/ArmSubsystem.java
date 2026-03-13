package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANIDConstants;
import frc.robot.utils.Logger;

public class ArmSubsystem extends SubsystemBase {

 
    public final SparkMax armMotor = new SparkMax(CANIDConstants.intakeArmID, MotorType.kBrushless);
    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.

    public static double gearReduction = 20.;
    public static double beltPulleyRatio = 1.5;

    public static double armLength = Units.inchesToMeters(20);
    public static double armMass = Units.lbsToKilograms(8.3);

    static double radperencderrev = (2 * Math.PI) / (beltPulleyRatio * gearReduction);

    public static double positionConversionFactor = radperencderrev;

    public static double velocityConversionFactor = positionConversionFactor / 60;

    static double maxmotorrps = 5700 / 60;

    public static double maxradpersec = radperencderrev * maxmotorrps;//

    double maxdegreespersec = Units.radiansToDegrees(maxradpersec);

    private static double kDt = 0.02;

    private static double kMaxVelocity = maxradpersec;;
    private static double kMaxAcceleration = 0.75;

    private static double kS = 1.1;
    private static double kG = 1.2;
    private static double kV = 12 / maxradpersec;

    private static double kP = 1.3;
    private static double kI = 0.0;
    private static double kD = 0.7;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
            kMaxAcceleration);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    private final Alert armAlert = new Alert(
            "Arm Fault",
            AlertType.kError);

    public ArmFeedforward armfeedforward;

    /**
     * Angles are set so that 90 degrees is with the arm balanced over center
     * This means kg will act equally on both sides of top center
     * 
     */
    public final static Angle minAngle = Degrees.of(-100);

    public final static Angle maxAngle = Degrees.of(40);

    double TRAJECTORY_VEL = 2 * Math.PI;
    double TRAJECTORY_ACCEL = 4 * Math.PI;

    public final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            TRAJECTORY_VEL, TRAJECTORY_ACCEL));

    private Angle upAngle = Degrees.of(-90);
    private Angle downAngle = Degrees.of(35);

    public boolean showData;

    public ArmSubsystem(boolean showData) {

        armMotor.configure(
                Configs.IntakeArm.armConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        armfeedforward = new ArmFeedforward(kS, kG, kV);

        armMotor.getEncoder().setPosition(upAngle.in(Radians));

        m_controller.setGoal(minAngle.in(Radians));

        armAlert.set(armMotor.hasActiveFault() || armMotor.hasStickyFault());

    }

    @Override
    public void periodic() {
        Logger.log("Intake/Angle", getCurrentAngle().in(Degrees));
        Logger.log("Intake/Velocity", getArmVelocity());
        Logger.log("Intake/Amps", armMotor.getOutputCurrent());
        Logger.log("Intake/Target", m_controller.getGoal().position);
    }

    @Override
    public void simulationPeriodic() {

    }

    public void resetEncoder(double val) {
        armMotor.getEncoder().setPosition(val);
    }

    public boolean inPosition() {
        return m_controller.atSetpoint();
    }

    public Command positionArmCommand() {
        return new FunctionalCommand(
                () -> {

                }, // init
                () -> {
                    armMotor.setVoltage(
                            m_controller.calculate(getCurrentAngle().in(Radians))
                                    + armfeedforward.calculate(m_controller.getSetpoint().position,
                                            m_controller.getSetpoint().velocity));
                }, // execute
                (interrupted) -> armMotor.set(0), // end
                () -> false, // isFinished
                this);// requirements
    }

    public Command intakeArmDownCommand() {
        return Commands.runOnce(() -> setControllerGoal(downAngle));
    }

    public Command intakeArmUpCommand() {
        return Commands.runOnce(() -> setControllerGoal(upAngle));
    }

    public Command jogIntakeArmCommand(DoubleSupplier jogRate) {
        return new FunctionalCommand(
                () -> {
                }, // init
                () -> {
                    if (getCurrentAngle().lt(maxAngle) && jogRate.getAsDouble() > 0)
                        armMotor.setVoltage(jogRate.getAsDouble() * RobotController.getBatteryVoltage());
                    else if (getCurrentAngle().gte(minAngle) && jogRate.getAsDouble() < 0)
                        armMotor.setVoltage(jogRate.getAsDouble() * RobotController.getBatteryVoltage());
                    else
                        armMotor.set(0);
                }, // execute
                (interrupted) -> armMotor.set(0), // end
                () -> false, // isFinished
                this);// requirements
    }

    public void setControllerGoal(Angle target) {
        m_controller.setGoal(target.in(Radians));
    }

    public Angle getCurrentAngle() {
        return Radians.of(armMotor.getEncoder().getPosition());
    }

    public double getArmVelocity() {
        return Units.radiansToDegrees(armMotor.getEncoder().getVelocity());
    }

    public void stop() {
        armMotor.setVoltage(0);
    }

    public double getAmps() {
        return armMotor.getOutputCurrent();
    }

    public boolean isBraked() {
        return armMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return armMotor.configAccessor.softLimit.getForwardSoftLimitEnabled()
                || armMotor.configAccessor.softLimit.getReverseSoftLimitEnabled();
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> armMotor.clearFaults());
    }

    public double getVolts() {
        return armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    }

}
