package frc.robot.subsystems;

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
    private double homePosition = 45;// degrees

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

        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1).feedForward
                // kV is now in Volts, so we multiply by the nominal voltage (12V)
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

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
        setPosition(homePosition);
        if (showData)
            SmartDashboard.putData(this);
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        targetPosition = clampedPosition;
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, encoder.getPosition(), kPositionTolerance);
    }

    private void closeLoop() {
        closedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    }

    @Override
    public void periodic() {
        closeLoop();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
                null);
        builder.addDoubleProperty("Current Position", () -> encoder.getPosition(), null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, null);
    }
}
