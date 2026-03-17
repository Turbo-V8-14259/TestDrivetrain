package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

/**
 * Pose2D is a convenience wrapper around the FTC SDK Pose2D.
 * <p>
 * It standardizes units to:
 * <ul>
 *     <li>x, y in inches</li>
 *     <li>heading in radians</li>
 * </ul>
 * <p>
 * Provides additional operations inspired by vector mathematics:
 * <ul>
 *     <li>Pose addition/subtraction</li>
 *     <li>Scalar multiplication/division</li>
 *     <li>Rotation and mirroring</li>
 *     <li>Conversion to/from Road Runner vectors and FTC Pose3D/Pose2D</li>
 * </ul>
 * <p>
 * All operations are immutable: they return a new Pose2D without modifying the original.
 */
public class Pose2D extends org.firstinspires.ftc.robotcore.external.navigation.Pose2D {

    /**
     * Creates a new Pose2D with the given x, y, and heading.
     * Units: inches for x/y, radians for heading.
     * Heading is stored unnormalized.
     *
     * @param x       x-coordinate in inches
     * @param y       y-coordinate in inches
     * @param heading heading in radians (unnormalized)
     */
    public Pose2D(double x, double y, double heading) {
        super(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading);
    }

    /** Default constructor: x=0, y=0, heading=0 */
    public Pose2D() {
        this(0, 0, 0);
    }

    /**
     * Creates a new Pose2D with a specified position and heading=0.
     *
     * @param x x-coordinate in inches
     * @param y y-coordinate in inches
     */
    public Pose2D(double x, double y) {
        this(x, y, 0.0);
    }

    /**
     * Constructs a Pose2D from an existing FTC Pose2D.
     * Converts units to inches/radians if needed.
     *
     * @param pose2D FTC SDK Pose2D
     */
    public Pose2D(org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose2D) {
        this(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH), pose2D.getHeading(AngleUnit.RADIANS));
    }

    /**
     * Constructs a Pose2D from an FTC Pose3D.
     * Extracts x/y and yaw only; z, pitch, roll are discarded.
     *
     * @param pose3D FTC SDK Pose3D
     */
    public Pose2D(Pose3D pose3D) {
        this(pose3D.getPosition().toUnit(DistanceUnit.INCH).x,
                pose3D.getPosition().toUnit(DistanceUnit.INCH).y,
                pose3D.getOrientation().getYaw(AngleUnit.RADIANS));
    }

    /**
     * Constructs a Pose2D from a VectorF.
     * Supports vectors of length 2 (x, y with heading=0) or length 3 (x, y, heading).
     * Units: inches for x/y, radians for heading.
     *
     * @param vector VectorF with length 2 or 3
     * @throws IllegalArgumentException if vector length is not 2 or 3
     */
    public Pose2D(VectorF vector) {
        this(vector.length() == 2 ? vector.get(0) : (vector.length() == 3 ? vector.get(0) : throwInvalidVectorLength(vector.length())),
             vector.length() == 2 ? vector.get(1) : (vector.length() == 3 ? vector.get(1) : 0),
             vector.length() == 3 ? vector.get(2) : 0);
    }

    private static double throwInvalidVectorLength(int length) {
        throw new IllegalArgumentException("VectorF must have length 2 (x, y) or 3 (x, y, heading), got length: " + length);
    }

    // ---------- Static helpers for symmetry with Kotlin ----------

    /** Scalar multiplication: returns `pose * scalar`. */
    public static Pose2D times(double scalar, Pose2D pose) {
        return pose.times(scalar);
    }

    /** Scalar division: returns `pose / scalar`. */
    public static Pose2D div(double scalar, Pose2D pose) {
        return pose.div(scalar);
    }

    // ---------- Accessors ----------

    /** Returns x-coordinate in inches. */
    public double getX() {
        return super.getX(DistanceUnit.INCH);
    }

    /** Returns y-coordinate in inches. */
    public double getY() {
        return super.getY(DistanceUnit.INCH);
    }

    /**
     * Returns heading in radians normalized to [-PI, PI].
     */
    public double getNormalizedHeading() {
        return getNormalizedHeading(AngleUnit.RADIANS);
    }

    /**
     * Returns heading normalized in the requested angle unit.
     *
     * @param unit desired unit (RADIANS or DEGREES)
     */
    public double getNormalizedHeading(AngleUnit unit) {
        return super.getHeading(unit);
    }

    /**
     * Returns the unnormalized heading in the requested unit.
     *
     * @param unit desired unnormalized angle unit
     * @return heading in requested unit without normalization
     */
    public double getHeading(UnnormalizedAngleUnit unit) {
        if (unit == UnnormalizedAngleUnit.DEGREES && headingUnit == AngleUnit.DEGREES) return heading;
        if (unit == UnnormalizedAngleUnit.RADIANS && headingUnit == AngleUnit.RADIANS) return heading;
        if (unit == UnnormalizedAngleUnit.DEGREES && headingUnit == AngleUnit.RADIANS) return Math.toDegrees(heading);
        if (unit == UnnormalizedAngleUnit.RADIANS && headingUnit == AngleUnit.DEGREES) return Math.toRadians(heading);
        return 0.0;
    }

    /** Returns the unnormalized heading in radians as stored. */
    public double getHeading() {
        return this.getHeading(UnnormalizedAngleUnit.RADIANS);
    }

    // ---------- Vector-style operations ----------

    /**
     * Component-wise addition of poses.
     * Operation: this + other
     *
     * @param other pose to add
     * @return new Pose2D representing this + other
     */
    public Pose2D plus(Pose2D other) {
        return new Pose2D(this.getX() + other.getX(), this.getY() + other.getY(), this.getHeading() + other.getHeading());
    }

    /**
     * Component-wise subtraction of poses.
     * Operation: this - other
     *
     * @param other pose to subtract
     * @return new Pose2D representing this - other
     */
    public Pose2D minus(Pose2D other) {
        return new Pose2D(this.getX() - other.getX(), this.getY() - other.getY(), this.getHeading() - other.getHeading());
    }

    /**
     * Component-wise scalar multiplication.
     * Operation: this * scalar
     *
     * @param scalar value to multiply by
     * @return new Pose2D representing this * scalar
     */
    public Pose2D times(double scalar) {
        return new Pose2D(this.getX() * scalar, this.getY() * scalar, this.getHeading() * scalar);
    }

    /**
     * Component-wise scalar division.
     * Operation: this / scalar
     *
     * @param scalar value to divide by
     * @return new Pose2D representing this / scalar
     */
    public Pose2D div(double scalar) {
        return new Pose2D(this.getX() / scalar, this.getY() / scalar, this.getHeading() / scalar);
    }

    /**
     * Rotation about the origin by a given angle.
     *
     * @param angle rotation in radians
     * @return new Pose2D representing this rotated by angle
     */
    public Pose2D rotated(double angle) {
        double x = getX();
        double y = getY();
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Pose2D(newX, newY, getHeading() + angle);
    }

    /**
     * Unary negation.
     * Operation: -this
     *
     * @return new Pose2D representing -this
     */
    public Pose2D unaryMinus() {
        return new Pose2D(-this.getX(), -this.getY(), -this.getHeading());
    }

    /**
     * Mirroring about the given axis
     *
     * @param axis axis to mirror around (X or Y)
     * @return new Pose2D mirrored across the axis
     * @throws IllegalArgumentException if axis is Z or UNKNOWN
     */
    public Pose2D mirrored(Axis axis) {
        if (axis == Axis.Z || axis == Axis.UNKNOWN)
            throw new IllegalArgumentException("Invalid axis set for mirroring of a Pose2D");

        double x = getX();
        double y = getY();
        double h = getHeading();

        if (axis == Axis.X) {
            return new Pose2D(x, -y, -h);
        } else {
            return new Pose2D(-x, y, Math.PI - h);
        }
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "x=%.3f in, y=%.3f in, heading=%.3f deg", getX(), getY(), getNormalizedHeading(AngleUnit.DEGREES));
    }
}
