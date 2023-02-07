package org.frc1778.lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import kotlin.math.sign

class ChargingStation(private val alliance: Alliance) {
    //Position of robot on the Charging Station Relative to the current AOR
    private var stationPose = Pose2d()


    //Current Angle of the table
    private var currAngle = 0.0

    //Offset to center of the charging station
    private var stationCenterXOffset = when (alliance) {
        Alliance.Red -> RED_STATION_CENTER_X_OFFSET
        else -> BLUE_STATION_CENTER_X_OFFSET
    }

    /**
     * The usable area of the charging station as a [Rectangle2d] representing the area of the charging station
     * projected onto the field plane
     */
    val chargingStationArea: Rectangle2d
        get() = Rectangle2d(
            //TODO: Translate corner points to reference POR
            when (alliance) {
                Alliance.Red -> RED_BOTTOM_LEFT
                else -> BLUE_BOTTOM_LEFT
            }.toTranslation3d().rotateBy(
                Rotation3d( 0.0, Math.toRadians(currAngle), 0.0)
            ).toTranslation2d(), when (alliance) {
                Alliance.Red -> RED_TOP_RIGHT
                else -> BLUE_TOP_RIGHT
            }.toTranslation3d().rotateBy(
                Rotation3d( 0.0, Math.toRadians(currAngle), 0.0)
            ).toTranslation2d()
        )

    /**
     * Update the charging station based on a transformation and a new angle
     *
     * @param transform2d the [Transform2d] representing the transformation the robot would make on flat ground
     * @param angle the new angle of the charging station
     * @return the field position of the robot on the platform
     */
    fun update(transform2d: Transform2d, angle: Double): Pose2d {
        //Move the stationPose based on the transform2d representing the change in robot pose from Drive Odometry
        stationPose = stationPose.transformBy(
            transform2d
        )
        // If the table changes tilt directions: transform the stationPose, so it references the correct AOR.
        // Has no offset when at zero
        if (angle.sign != currAngle.sign) {
            stationPose = stationPose.transformBy(
                Transform2d(
                    Translation2d(angle.sign * RIGHT_TO_LEFT_POR_TRANSLATION, 0.0), Rotation2d()
                )
            )
        }
        /**
         * Perform rotation to convert from station plane to field plane.
         *  - Convert the station pose into a [Pose3d] and rotate its translation component by the difference in the
         *  held angle value([currAngle]) and the new angle.
         *  - Create a new [Pose2d] object with this rotated translation and the current yaw of the
         *  station pose(this should always be the same as the odometry)
         *  - Transform the [Pose2d] by the station offset and the AOR offset to bring the position back to
         *  referencing the field instead of the station
         */
        val fieldPose: Pose2d = Pose2d(
            Pose3d(stationPose).translation.rotateBy(Rotation3d(0.0, Math.toRadians(angle), 0.0))
                .toTranslation2d(), Rotation2d(stationPose.rotation.radians)
        ).transformBy(
            Transform2d(
                Translation2d(stationCenterXOffset + (angle.sign * RIGHT_TO_LEFT_POR_TRANSLATION), POR_Y_OFFSET),
                Rotation2d()
            )
        )

        //Change the held angle to the new angle
        currAngle = angle
        return fieldPose
    }

    /**
     * Helper function to convert a [Translation2d] into a [Translation3d]
     * @return [Translation3d]
     */
    private fun Translation2d.toTranslation3d() = Translation3d(
        this.x, this.y, 0.0
    )

    /**
     * Set the station pose based on the robots current field pose. This will be helpful
     * when moving to and from zones where we want to track station pose
     * @param fieldPose current field position of the robot
     */
    fun setPose(fieldPose: Pose2d) {
        stationPose = fieldPose.transformBy(
            Transform2d(
                Translation2d(-(stationCenterXOffset + currAngle.sign * RIGHT_TO_LEFT_POR_TRANSLATION), -POR_Y_OFFSET),
                Rotation2d()
            )
        )
    }

    companion object {
        private const val POR_Y_OFFSET = 0.0

        private const val RIGHT_TO_LEFT_POR_TRANSLATION = 0.0

        private val BLUE_BOTTOM_LEFT: Translation2d = Translation2d(0.0, 0.0)
        private val BLUE_TOP_RIGHT: Translation2d = Translation2d(0.0, 0.0)
        private val RED_BOTTOM_LEFT: Translation2d = Translation2d(0.0, 0.0)
        private val RED_TOP_RIGHT: Translation2d = Translation2d(0.0, 0.0)

        private const val BLUE_STATION_CENTER_X_OFFSET = 0.0
        private const val RED_STATION_CENTER_X_OFFSET = 0.0
    }
}