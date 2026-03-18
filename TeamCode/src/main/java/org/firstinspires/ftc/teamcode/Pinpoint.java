package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
public class Pinpoint implements Localizer {
    public static double xOffset = 133;
    public static double yOffset = -79;
    private final GoBildaPinpointDriver pinpoint;
    private Pose2D startPos;
    private Pose2D robotPos = new Pose2D();
    private Pose2D worldOffset = new Pose2D();
    private Pose2D robotVelocity = new Pose2D();

    public Pinpoint(HardwareMap hardwareMap, Pose2D startPos) throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        configure(xOffset, yOffset, DistanceUnit.MM);

        this.startPos = startPos;
        pinpoint.recalibrateIMU();
        Thread.sleep(500);
        // Note: add a wait before this if you call the other function, because it halts the position settings.
        setPosition(startPos);
    }

    // Set up with no starting position.
    public Pinpoint(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        configure(xOffset, yOffset, DistanceUnit.MM);
        pinpoint.recalibrateIMU();
    }

    @Override
    public Pose2D getStartPos() {
        return startPos;
    }

    @Override
    public void setPosition(@NonNull Pose2D position) {
        pinpoint.setPosX(position.getX(DistanceUnit.INCH), DistanceUnit.INCH);
        pinpoint.setPosY(position.getY(DistanceUnit.INCH), DistanceUnit.INCH);
        pinpoint.setHeading(position.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);
    }

    @Override
    public void setPositionTranslational(@NonNull Pose2D position) {
        pinpoint.setPosX(position.getX(DistanceUnit.INCH), DistanceUnit.INCH);
        pinpoint.setPosY(position.getY(DistanceUnit.INCH), DistanceUnit.INCH);
    }

    private void configure(double xOffset, double yOffset, DistanceUnit units) {
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(xOffset, yOffset, units);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary.
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        // pinpoint.resetPosAndIMU();
    }

    @Override
    public void addOffset(@NonNull Pose2D offset) {
        worldOffset.plus(offset);
    }

    @Override
    public void setOffset(@NonNull Pose2D offset) {
        worldOffset = offset;
    }

    @Override
    public void read() {
        pinpoint.update();
        robotPos = new Pose2D(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS));
        robotVelocity = new Pose2D(
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }

    @Override
    public Pose2D getRobotPos() {
        return robotPos;
    }

    @Override
    public Pose2D getRobotVelocity() {
        return robotVelocity;
    }

    @Override
    public void resetPosAndIMU() {
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }
}
