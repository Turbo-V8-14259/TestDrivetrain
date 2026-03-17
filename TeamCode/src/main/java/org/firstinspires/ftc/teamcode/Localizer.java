package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.core.util.Pose2D;

public interface Localizer {
    Pose2D getStartPos();

    void setPosition(@NonNull Pose2D position);

    void setPositionTranslational(@NonNull Pose2D position);

    void addOffset(@NonNull Pose2D offset);

    void setOffset(@NonNull Pose2D offset);

    void read();

    Pose2D getRobotPos();

    Pose2D getRobotVelocity();

    void resetPosAndIMU();

    void recalibrateIMU();
}
