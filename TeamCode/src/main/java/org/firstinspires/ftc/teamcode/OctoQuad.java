package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class OctoQuad implements Localizer {
    // Configuration parameters - these can be tuned via FTC Dashboard
    public static int portX = 0;
    public static int portY = 1;
    public static OctoQuadFWv3.EncoderDirection directionX = OctoQuadFWv3.EncoderDirection.REVERSE;
    public static OctoQuadFWv3.EncoderDirection directionY = OctoQuadFWv3.EncoderDirection.REVERSE;
    //public static float ticksPerMM_X = 20.0103f;
    //public static float ticksPerMM_Y = 19.9639f;
    public static float ticksPerMM_X = 19.89436789f;
    public static float ticksPerMM_Y = 19.89436789f;
    public static float tcpOffsetMM_X = 85;  // Tracking center point offset X
    public static float tcpOffsetMM_Y = -126;  // Tracking center point offset Y
    public static float headingScalar = 1.0221f;  // IMU heading scalar
    public static int velocityIntervalMs = 25;  // Velocity measurement interval (1-255ms)

    private final OctoQuadFWv3 octoQuad;
    public  static double lastHeading;
    public static double wraps;
    private Pose2D startPos;
    private Pose2D robotPos = new Pose2D();
    private Pose2D worldOffset = new Pose2D();
    private Pose2D robotVelocity = new Pose2D();
    private final OctoQuadFWv3.LocalizerDataBlock localizerData = new OctoQuadFWv3.LocalizerDataBlock();

    // Conversion constants
    private static final double MM_TO_INCHES = 1.0 / 25.4;

    public OctoQuad(HardwareMap hardwareMap, Pose2D startPos) throws InterruptedException {
        octoQuad = hardwareMap.get(OctoQuadFWv3.class, "pinpoint");

        configure();

        this.startPos = startPos;

        // Wait for localizer to be ready (similar to test file pattern)
        // This ensures IMU calibration is complete
        int maxWaitMs = 5000; // 5 second timeout
        int waitedMs = 0;
        while (waitedMs < maxWaitMs) {
            OctoQuadFWv3.LocalizerStatus status = octoQuad.getLocalizerStatus();
            if (status == OctoQuadFWv3.LocalizerStatus.RUNNING) {
                break;
            }
            Thread.sleep(100);
            waitedMs += 100;
        }

        // Set the starting position
        setPosition(startPos);
    }

    // Set up with no starting position
    public OctoQuad(HardwareMap hardwareMap) throws InterruptedException {
        octoQuad = hardwareMap.get(OctoQuadFWv3.class, "pinpoint");

        configure();

        // Wait for localizer to be ready (similar to test file pattern)
        // This ensures IMU calibration is complete
        int maxWaitMs = 5000; // 5 second timeout
        int waitedMs = 0;
        while (waitedMs < maxWaitMs) {
            OctoQuadFWv3.LocalizerStatus status = octoQuad.getLocalizerStatus();
            if (status == OctoQuadFWv3.LocalizerStatus.RUNNING) {
                break;
            }
            Thread.sleep(100);
            waitedMs += 100;
        }
    }

    private void configure() {
        // Set encoder directions
        octoQuad.setSingleEncoderDirection(portX, directionX);
        octoQuad.setSingleEncoderDirection(portY, directionY);

        // Configure all localizer parameters
        octoQuad.setAllLocalizerParameters(
                portX,
                portY,
                ticksPerMM_X,
                ticksPerMM_Y,
                tcpOffsetMM_X,
                tcpOffsetMM_Y,
                headingScalar,
                velocityIntervalMs
        );

        // Set I2C recovery mode for better reliability
        octoQuad.setI2cRecoveryMode(OctoQuadFWv3.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
    }

    @Override
    public Pose2D getStartPos() {
        return startPos;
    }

    @Override
    public void setPosition(@NonNull Pose2D position) {
        // Convert inches to mm for OctoQuad
        int posX_mm = (int) (position.getX(DistanceUnit.INCH) / MM_TO_INCHES);
        int posY_mm = (int) (position.getY(DistanceUnit.INCH) / MM_TO_INCHES);
        float heading_rad = (float) position.getHeading(AngleUnit.RADIANS);
        octoQuad.setLocalizerPose(posX_mm, posY_mm, heading_rad);
    }

    @Override
    public void setPositionTranslational(@NonNull Pose2D position) {
        // Convert inches to mm for OctoQuad
        int posX_mm = (int) (position.getX(DistanceUnit.INCH) / MM_TO_INCHES);
        int posY_mm = (int) (position.getY(DistanceUnit.INCH) / MM_TO_INCHES);
        float currentHeading = localizerData.heading_rad;

        octoQuad.setLocalizerPose(posX_mm, posY_mm, currentHeading);
    }

    @Override
    public void addOffset(@NonNull Pose2D offset) {
        worldOffset = worldOffset.plus(offset);
    }

    @Override
    public void setOffset(@NonNull Pose2D offset) {
        worldOffset = offset;
    }

    @Override
    public void read() {
        // Read the localizer data from the OctoQuad
        octoQuad.readLocalizerData(localizerData);

        // Only update robot pose if data is valid
        if (localizerData.isPoseDataValid()) {
            // Convert mm to inches
            robotPos = new Pose2D(
                    localizerData.posX_mm * MM_TO_INCHES,
                    localizerData.posY_mm * MM_TO_INCHES,
                    localizerData.heading_rad
            );

            // Convert mm/s to inches/s
            robotVelocity = new Pose2D(
                    localizerData.velX_mmS * MM_TO_INCHES,
                    localizerData.velY_mmS * MM_TO_INCHES,
                    localizerData.velHeading_radS
            );
            if(Math.abs(lastHeading - robotPos.getHeading()) >= Math.PI){
                wraps += 1 * Math.signum(lastHeading - robotPos.getHeading());
            }
        }
    }

    @Override
    public Pose2D getRobotPos() {
        return robotPos.plus(worldOffset);
    }

    @Override
    public Pose2D getRobotVelocity() {
        return robotVelocity;
    }

    @Override
    public void resetPosAndIMU() {
        octoQuad.resetLocalizerAndCalibrateIMU();
    }

    @Override
    public void recalibrateIMU() {
        
    }

    /**
     * Get the current status of the localizer
     * @return the localizer status
     */
    public OctoQuadFWv3.LocalizerStatus getStatus() {
        return localizerData.localizerStatus;
    }

    /**
     * Check if the localizer is running and ready
     * @return true if the localizer is running
     */
    public boolean isRunning() {
        return localizerData.localizerStatus == OctoQuadFWv3.LocalizerStatus.RUNNING;
    }

    /**
     * Check if the last data read had a valid CRC
     * @return true if CRC was valid
     */
    public boolean isCrcOk() {
        return localizerData.crcOk;
    }

    /**
     * Get the firmware version string from the OctoQuad
     * @return firmware version string
     */
    public String getFirmwareVersion() {
        return octoQuad.getFirmwareVersionString();
    }

    /**
     * Get which IMU axis was selected for heading
     * @return the yaw axis choice
     */
    public OctoQuadFWv3.LocalizerYawAxis getHeadingAxisChoice() {
        return octoQuad.getLocalizerHeadingAxisChoice();
    }

    /**
     * Set the heading only (teleport heading without changing position)
     * @param headingRad heading in radians
     */
    public void setHeading(double headingRad) {
        octoQuad.setLocalizerHeading((float) headingRad);
    }
}
