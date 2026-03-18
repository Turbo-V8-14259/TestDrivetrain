package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Drivetrain {
    public static Pose2D publicRobotLocation = new Pose2D(0, 0, 0);
    public static Pose2D publicRobotVelocity = new Pose2D(0, 0, 0);
    public static double maxTranslationalPower = 1;

    //private final Localizer odo;
    private final DcMotorEx FL;
    private final DcMotorEx BL;
    private final DcMotorEx FR;
    private final DcMotorEx BR;

    private Pose2D robotPosition;
    private Pose2D robotVelocity = new Pose2D();

    private boolean powerScaled = true;
    private double maxDTPower = 1;
    private double lastFrontLeftPower = 0;
    private double lastFrontRightPower = 0;
    private double lastBackLeftPower = 0;
    private double lastBackRightPower = 0;

    public Drivetrain(HardwareMap hardwareMap, Pose2D startPosition) throws InterruptedException {
        FL = hardwareMap.get(DcMotorEx.class, "fl");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        FR = hardwareMap.get(DcMotorEx.class, "fr");
        BR = hardwareMap.get(DcMotorEx.class, "br");

        //odo = new OctoQuad(hardwareMap, startPosition);
        robotPosition = startPosition;

        configureDriveMotors();
        //Systems.DT.setSubSystem(this);
    }

    public Drivetrain(HardwareMap hardwareMap) throws InterruptedException {
        FL = hardwareMap.get(DcMotorEx.class, "fl");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        FR = hardwareMap.get(DcMotorEx.class, "fr");
        BR = hardwareMap.get(DcMotorEx.class, "br");

        //odo = new OctoQuad(hardwareMap);
        //robotPosition = odo.getRobotPos();

        configureDriveMotors();
        //Systems.DT.setSubSystem(this);
    }

    private void configureDriveMotors() {
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void powerScale(boolean powerScaled) {
        this.powerScaled = powerScaled;
    }

    public void setMaxDTPower(double maxDTPower) {
        this.maxDTPower = Math.max(0, Math.min(1, this.maxDTPower));
    }

    public void setPowersFieldCentric(double strafe, double forward, double rx) {
        double heading = robotPosition.getHeading();
        Vector2d robotVec = new Vector2d(strafe, forward).rotated(heading - Math.PI / 2);

        setPowers(robotVec.getX(), robotVec.getY(), rx);
    }

    public void setPowers(double strafe, double forward, double rx) {
        rx = -rx;

        double moveMaxPower = Math.min(maxTranslationalPower, 1 - Math.abs(rx));
        if (powerScaled) {
            double v = Math.abs(strafe) + Math.abs(forward);
            if (v > moveMaxPower) {
                double scale = moveMaxPower / v;
                strafe *= scale;
                forward *= scale;
            }
        }

        double frontLeftPower = forward - strafe - rx;
        double frontRightPower = forward + strafe + rx;
        double backLeftPower = forward + strafe - rx;
        double backRightPower = forward - strafe + rx;

        frontLeftPower = cap(applyDeadzone(frontLeftPower));
        frontRightPower = cap(applyDeadzone(frontRightPower));
        backLeftPower = cap(applyDeadzone(backLeftPower));
        backRightPower = cap(applyDeadzone(backRightPower));

        if (Math.abs(frontLeftPower - lastFrontLeftPower) >= 0.03) {
            FL.setPower(frontLeftPower);
            lastFrontLeftPower = frontLeftPower;
        }
        if (Math.abs(frontRightPower - lastFrontRightPower) >= 0.03) {
            FR.setPower(frontRightPower);
            lastFrontRightPower = frontRightPower;
        }
        if (Math.abs(backLeftPower - lastBackLeftPower) >= 0.03) {
            BL.setPower(backLeftPower);
            lastBackLeftPower = backLeftPower;
        }
        if (Math.abs(backRightPower - lastBackRightPower) >= 0.03) {
            BR.setPower(backRightPower);
            lastBackRightPower = backRightPower;
        }
    }

//    public Pose2D updateOdometry() {
//        odo.read();
//        robotPosition = new Pose2D(odo.getRobotPos().getX(), odo.getRobotPos().getY(), odo.getRobotPos().getHeading());
//        robotVelocity = odo.getRobotVelocity();
//        publicRobotLocation = robotPosition;
//        publicRobotVelocity = robotVelocity;
//        return robotPosition;
//    }
//
//    public Pose2D getRobotPosition() {
//        robotPosition = new Pose2D(odo.getRobotPos().getX(), odo.getRobotPos().getY(), odo.getRobotPos().getHeading());
//        publicRobotLocation = robotPosition;
//        return robotPosition;
//    }
//
//    public Pose2D getRobotVelocity() {
//        robotVelocity = odo.getRobotVelocity();
//        publicRobotVelocity = robotVelocity;
//        return robotVelocity;
//    }
//
//    public Localizer getLocalizer() {
//        return odo;
//    }
//
//    public void recalibrateIMU() {
//        odo.recalibrateIMU();
//    }
//
//    public void resetPosAndIMU() {
//        odo.resetPosAndIMU();
//    }
//
//    public void setPosition(Pose2D position) {
//        odo.setPosition(new Pose2D(position.getX(), position.getY(), robotPosition.getHeading(UnnormalizedAngleUnit.RADIANS)));
//    }
//
//    public void setPositionTranslational(Pose2D position) {
//        odo.setPositionTranslational(new Pose2D(position.getX(), position.getY(), 0));
//    }
//
//    public void hardResetClose(boolean red) {
//        if (red) {
//            odo.setPosition(new Pose2D(32.5, 63.5, Math.PI / 2));
//        } else {
//            odo.setPosition(new Pose2D(-32.5, 63.5, Math.PI / 2));
//        }
//    }
//
//    public void hardResetHP(boolean red) {
//        if (red) {
//            odo.setPosition(new Pose2D(-63, -63.5, Math.PI / 2));
//        } else {
//            odo.setPosition(new Pose2D(63, -63.5, Math.PI / 2));
//        }
//    }

    public double getCurrentDraw() {
        return FL.getCurrent(CurrentUnit.AMPS) + BL.getCurrent(CurrentUnit.AMPS) + FR.getCurrent(CurrentUnit.AMPS) + BR.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isInZone(double robotLength, double robotWidth) {
        return isInZone(robotLength, robotWidth, robotPosition);
    }

    public static boolean isInZone(double robotLength, double robotWidth, Pose2D robotPosition) {
        double halfLength = robotLength / 2.0;
        double halfWidth = robotWidth / 2.0;

        Pose2D[] localCorners = localCorners(halfWidth, halfLength);
        Pose2D[] worldCorners = worldCorners(localCorners, robotPosition.getHeading(), new Pose2D(robotPosition.getX(), robotPosition.getY()));

        for (Pose2D corner : worldCorners) {
            if (isCornerInZone(corner)) {
                return true;
            }
        }

        for (int i = 0; i < 4; i++) {
            Pose2D a = worldCorners[i];
            Pose2D b = worldCorners[(i + 1) % 4];
            if (edgeIntersectsZone(a, b)) {
                return true;
            }
        }

        return false;
    }

    private double applyDeadzone(double power) {
        if (Math.abs(power) < 0.03) {
            return 0;
        }
        return power;
    }

    private double cap(double power) {
        return Math.min(Math.abs(power), maxDTPower) * Math.signum(power);
    }

    private static Pose2D[] localCorners(double halfWidth, double halfLength) {
        return new Pose2D[]{new Pose2D(halfWidth, halfLength), new Pose2D(-halfWidth, halfLength), new Pose2D(-halfWidth, -halfLength), new Pose2D(halfWidth, -halfLength)};
    }

    private static Pose2D[] worldCorners(Pose2D[] localCorners, double heading, Pose2D offset) {
        Pose2D[] out = new Pose2D[localCorners.length];
        for (int i = 0; i < localCorners.length; i++) {
            out[i] = localCorners[i].rotated(heading).plus(offset);
        }
        return out;
    }

    private static boolean isCornerInZone(Pose2D p) {
        double x = p.getX();
        double y = p.getY();
        return (y >= Math.abs(x)) || (y <= -Math.abs(x) - 48);
    }

    private static boolean edgeIntersectsZone(Pose2D a, Pose2D b) {
        if (segmentIntersectsBoundary(a, b, 1.0, 0.0, true)) return true;
        if (segmentIntersectsBoundary(a, b, -1.0, 0.0, false)) return true;
        if (segmentIntersectsBoundary(a, b, -1.0, -48.0, true)) return true;
        return segmentIntersectsBoundary(a, b, 1.0, -48.0, false);
    }

    private static boolean segmentIntersectsBoundary(Pose2D a, Pose2D b, double m, double c, boolean positiveSide) {
        double ax = a.getX();
        double ay = a.getY();
        double bx = b.getX();
        double by = b.getY();
        double fa = ay - m * ax - c;
        double fb = by - m * bx - c;

        if (fa * fb > 0) {
            return false;
        }

        double dx = bx - ax;
        double dy = by - ay;
        double denom = dy - m * dx;

        if (Math.abs(denom) < 1e-6) {
            if (Math.abs(fa) < 1e-6) {
                double xMin = Math.min(ax, bx);
                double xMax = Math.max(ax, bx);
                return positiveSide ? xMax >= 0 : xMin <= 0;
            }
            return false;
        }

        double t = (m * ax + c - ay) / denom;
        if (t < 0 || t > 1) {
            return false;
        }

        double px = ax + t * dx;
        return positiveSide ? px >= 0 : px <= 0;
    }
}
