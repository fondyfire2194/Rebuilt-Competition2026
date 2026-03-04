package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.utils.Logger;

public class HoodSubsystem extends SubsystemBase {
    public static final Angle kMinPosition = Degrees.of(0.0);
    public static final Angle kMaxPosition = Degrees.of(10.);

    private static final double kPositionTolerance = 0.01;

    private static double manualTargetAngle;
    public static double autoTargetAngle;
    private static double finalTargetAngle;

    private final SparkMax hoodMotor;

    private final static double gearRatio = 139;

    private final static double pinionTeeth = 30;

    private final static double quadrantDegreesPerTooth = 360. / 350;// approx 1.3

    private final static double degreesPerPinionRev = pinionTeeth * quadrantDegreesPerTooth;// approx 40

    public final static double degreesPerMotorRev = degreesPerPinionRev / gearRatio;// approx .22

    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    public boolean showData;

    private int tst;
    private boolean hoodUsingDistance;

    public boolean isHoodUsingDistance() {
        return hoodUsingDistance;
    }

    public void setHoodUsingDistance(boolean autoHood) {
        this.hoodUsingDistance = autoHood;
        finalTargetAngle = isHoodUsingDistance() ? autoTargetAngle : manualTargetAngle;

    }

    public Command setHoodUsingDistanceCommand(boolean on) {
        return Commands.sequence(
                Commands.runOnce(() -> setHoodUsingDistance(on)),
                Commands.runOnce(
                        () -> finalTargetAngle = isHoodUsingDistance() ? autoTargetAngle : manualTargetAngle));

    }

    public HoodSubsystem(boolean showData) {
        hoodMotor = new SparkMax(Constants.CANIDConstants.hoodMotorID, MotorType.kBrushless);
        closedLoopController = hoodMotor.getClosedLoopController();
        encoder = hoodMotor.getEncoder();
        /*
         * Create a new SPARK MAX configuration object. This will store the
         * configuration parameters for the SPARK MAX that we will set below.
         */

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
        hoodMotor.configure(
                Configs.Hood.hoodConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        encoder.setPosition(kMinPosition.in(Degrees));

        this.showData = showData;
        if (showData)
            SmartDashboard.putData(this);
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(finalTargetAngle, getHoodPosition(), kPositionTolerance);
    }

    public Command positionToHomeCommand() {
        return Commands.runOnce(() -> finalTargetAngle = kMinPosition.in((Degrees)));
    }

    public Command positionTestCommand() {
        return Commands.runOnce(() -> finalTargetAngle = kMinPosition.in(Degrees) + 5);
    }

    public Command positionHoodCommand() {
        return new FunctionalCommand(
                () -> {
                    finalTargetAngle = getHoodPosition();
                    manualTargetAngle = finalTargetAngle;
                    autoTargetAngle = getHoodPosition();
                }, // init
                () -> {
                    closedLoopController.setSetpoint(finalTargetAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                }, // execute
                (interrupted) -> hoodMotor.set(0), // end
                () -> false, // isFinished
                this);// requirements
    }

    public void incrementHoodPosition(double val) {
        double pos = finalTargetAngle + val;
        pos = MathUtil.clamp(pos, kMinPosition.in(Degrees), kMaxPosition.in(Degrees));
        finalTargetAngle = pos;
    }

    public Command incrementHoodCommand(double val) {
        return Commands.runOnce(() -> incrementHoodPosition(val));
    }

    public Command setManualTargetCommand(double position) {
        return Commands.runOnce(() -> manualTargetAngle = position);
    }

    public double getHoodPosition() {
        return hoodMotor.getEncoder().getPosition();
    }

    public void runHoodMotor(double power) {
        hoodMotor.set(power);
    }

    public void stopHoodMotor() {
        hoodMotor.set(0);
    }

    public Command jogHoodUpCommand() {
        return this.startEnd(
                () -> {
                    if (getHoodPosition() < kMaxPosition.in(Degrees))
                        this.runHoodMotor(Constants.HoodSetpoints.jogHoodMotor);
                    manualTargetAngle = getHoodPosition();
                }, () -> {
                    this.runHoodMotor(0.0);
                }).withName("JogHoodUp");
    }

    public Command jogHoodDownCommand() {
        return this.startEnd(
                () -> {
                    if (getHoodPosition() > kMinPosition.in(Degrees))
                        this.runHoodMotor(-Constants.HoodSetpoints.jogHoodMotor);
                    manualTargetAngle = getHoodPosition();
                }, () -> {
                    this.runHoodMotor(0.0);
                }).withName("JogHoodDown");
    }

    public boolean getForwardSoftLimit(SparkMax motor) {
        return motor.getForwardSoftLimit().isReached();
    }

    public boolean getReverseSoftLimit(SparkMax motor) {
        return motor.getReverseSoftLimit().isReached();
    }

    @Override
    public void periodic() {
        Logger.log("Hood/FinalTargetAngle", finalTargetAngle);
        Logger.log("Hood/ManualTargetAngle", manualTargetAngle);
        Logger.log("Hood/AutoTargetAngle", autoTargetAngle);
        Logger.log("Hood/UseAutoTarget", isHoodUsingDistance());
        Logger.log("Hood/CurrentAngle", getHoodPosition());
        Logger.log("Hood/AngleError", finalTargetAngle - getHoodPosition());
        Logger.log("Hood/AtTarget", isPositionWithinTolerance());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
                null);
        builder.addDoubleProperty("Current Angle", () -> encoder.getPosition(), null);

        builder.addDoubleProperty("Manual Target Angle", () -> manualTargetAngle, null);
        builder.addDoubleProperty("Auto Target Angle", () -> autoTargetAngle, null);
        builder.addDoubleProperty("Final Target Angle", () -> finalTargetAngle, null);
        builder.addBooleanProperty("Use Distance For Angle", () -> isHoodUsingDistance(), null);
        builder.addDoubleProperty("Motor Amps", () -> hoodMotor.getOutputCurrent(), null);
        builder.addDoubleProperty("Motor Output Volts", () -> hoodMotor.getAppliedOutput() * 12, null);
        builder.addBooleanProperty("MaxTravelLimitReached", (() -> getForwardSoftLimit(hoodMotor)), null);
        builder.addBooleanProperty("MinTravelLimitReached", (() -> getReverseSoftLimit(hoodMotor)), null);

    }
}
