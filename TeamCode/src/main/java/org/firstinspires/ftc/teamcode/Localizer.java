package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;


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
