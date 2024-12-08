package util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import util.filters.ClampFilter;
import util.filters.DeadbandFilter;
import util.filters.ExponentialSmoothingFilter;
import util.filters.FilterSeries;
import util.filters.PowerFilter;
import util.filters.ScaleFilter;
import util.filters.WrapperFilter;

/**
 * New driver filter for swerve drive teleop input. Accepts a value in [-1, 1]
 */
public class OldDriverFilter2 extends FilterSeries {
    private SlewRateLimiter slewRateLimiter;
    private boolean belowDeadband;
    private final double deadbandScaler;

    /**
     * Construct a new driver filter
     * @param deadband
     * @param motorDeadband
     * @param scale
     * @param alpha
     * @param posRateLimit
     * @param negRateLimit
     */
    public OldDriverFilter2(double deadband, double motorDeadband, double scale,
                            double alpha, double posRateLimit, double negRateLimit) {
        super();

        deadbandScaler = (1 - motorDeadband) * (1 - motorDeadband);

        slewRateLimiter = new SlewRateLimiter(posRateLimit, negRateLimit, 0);

        super.setFilters(
                new DeadbandFilter(deadband),
                new ScaleFilter(1.7),
                new ClampFilter(1),
                new WrapperFilter(
                        (x) -> {
                            if (x == 0) {
                                belowDeadband = true;
                                return 0.0;
                            }
                            if (x > 0) {
                                belowDeadband = false;
                                return x - deadband;
                            } else {
                                belowDeadband = false;
                                return x + deadband;
                            }
                        }
                ),
                new ExponentialSmoothingFilter(alpha),
                new PowerFilter(3),
                new WrapperFilter(
                        (x) -> {
                            if (belowDeadband) return 0.0;
                            else {
                                return Math.signum(x) * ((Math.abs(x) / deadbandScaler) + motorDeadband);
                            }
                        }
                ),
                new WrapperFilter(
                        (x) -> {
                            return Math.signum(x)
                                    * slewRateLimiter.calculate(Math.abs(x));
                        }
                ),
                new ScaleFilter(scale),
                new ClampFilter(scale)
        );
    }
}
