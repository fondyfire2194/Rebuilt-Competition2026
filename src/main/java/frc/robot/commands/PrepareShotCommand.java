package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.ejml.dense.row.misc.RrefGaussJordanRowPivot_DDRM;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;

public class PrepareShotCommand extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
            (startValue, endValue, q) -> InverseInterpolator.forDouble()
                    .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
            (startValue, endValue, t) -> new Shot(
                    Interpolator.forDouble()
                            .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                    Interpolator.forDouble()
                            .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)));

    static {
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 45));
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 50));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 55));
    }

    private final TripleShooterSubsystem shooter;
    private final HoodSubsystem hood;

    public PrepareShotCommand(TripleShooterSubsystem shooter, HoodSubsystem hood) {
        this.shooter = shooter;
        this.hood = hood;
        addRequirements(shooter, hood);
    }

    public boolean isReadyToShoot() {
        return shooter.allVelocityInTolerance() && hood.isPositionWithinTolerance();
    }

    @Override
    public void execute() {
        final Distance distanceToHub = shooter.getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        shooter.setTargetVelocity(shot.shooterRPM);
        shooter.runAllVelocityVoltage();
        hood.setPosition(shot.hoodPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
