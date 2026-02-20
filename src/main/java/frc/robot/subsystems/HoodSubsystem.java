package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private static final double kMinPosition = 0.0;
    private static final double kMaxPosition = 20;
    private static final double kPositionTolerance = 0.01;

    private static double targetPosition;

    private final SparkMax hoodMotor;

    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private double degreesPerEncoderRev = 3.6;

    public boolean showData;

    public HoodSubsystem(boolean showData) {
        hoodMotor = new SparkMax(Constants.CANIDConstants.hoodMotorID, MotorType.kBrushless);
        closedLoopController = hoodMotor.getClosedLoopController();
        encoder = hoodMotor.getEncoder();
        /*
         * Create a new SPARK MAX configuration object. This will store the
         * configuration parameters for the SPARK MAX that we will set below.
         */
        motorConfig = new SparkMaxConfig();

        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        motorConfig.encoder
                .positionConversionFactor(degreesPerEncoderRev)
                .velocityConversionFactor(degreesPerEncoderRev / 60);

        motorConfig.softLimit
                .forwardSoftLimit(kMaxPosition)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(kMinPosition)
                .reverseSoftLimitEnabled(true);

        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.05)
                .i(0)
                .d(0)
                .outputRange(-.2, .25);

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
        hoodMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder.setPosition(kMinPosition);
        this.showData = showData;
        if (showData)
            SmartDashboard.putData(this);
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, getHoodPosition(), kPositionTolerance);
    }

    public Command positionToHomeCommand() {
        return Commands.runOnce(() -> targetPosition = kMinPosition);
    }

    public Command positionTestCommand() {
        return Commands.runOnce(() -> targetPosition = kMinPosition + 10);
    }

    public Command positionHoodCommand() {
        return new FunctionalCommand(
                () -> {
                    targetPosition = getHoodPosition();
                }, // init
                () -> {
                    closedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                }, // execute
                (interrupted) -> hoodMotor.set(0), // end
                () -> false, // isFinished
                this);// requirements
    }

    public void setTargetPosition(double position) {

        MathUtil.clamp(targetPosition, kMinPosition, kMaxPosition);
    }

    public Command setTargetCommand(double position) {
        return Commands.runOnce(() -> setTargetPosition(position));
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
                    if (getHoodPosition() > kMinPosition)
                        this.runHoodMotor(-Constants.HoodSetpoints.jogHoodMotor);
                    targetPosition = getHoodPosition();
                }, () -> {
                    this.runHoodMotor(0.0);
                }).withName("JogHoodUp");
    }

    public Command jogHoodDownCommand() {
        return this.startEnd(
                () -> {
                    if (getHoodPosition() < kMaxPosition)
                        this.runHoodMotor(Constants.HoodSetpoints.jogHoodMotor);
                    targetPosition = getHoodPosition();
                }, () -> {
                    this.runHoodMotor(0.0);
                }).withName("JogHoodDown");
    }

    @Override
    public void periodic() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
                null);
        builder.addDoubleProperty("Current Position", () -> encoder.getPosition(), null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, null);
        builder.addDoubleProperty("Motor Amps", () -> hoodMotor.getOutputCurrent(), null);

        builder.addBooleanProperty("MaxTravelLimitReached", (() -> getHoodPosition() >= kMaxPosition), null);
        builder.addBooleanProperty("MinTravelLimitReached", (() -> getHoodPosition() <= kMinPosition), null);

    }
}
