package org.frc1778

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import org.frc1778.commands.shooter.ShooterAngleCommand
import org.frc1778.subsystems.Drive
import org.frc1778.subsystems.Gyro
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class ShooterAngleCommandTests {

    fun testScenario(angle: SIUnit<Radian>, alliance: DriverStation.Alliance, shouldBeFront: Boolean) {

        var cmd = ShooterAngleCommand(Level.Middle)

        Robot.alliance = alliance
        Drive.robotPosition = Pose2d(0.0, 0.0, Rotation2d(angle.value))

        assertEquals(alliance, Robot.alliance)
        assertEquals(angle.value, Gyro.odometryYaw(), 0.01)

        cmd.initialize()
        if(shouldBeFront) {
            assertEquals(Level.Middle.frontShooterPosition, cmd.endPos)
        }else {
            assertEquals(Level.Middle.rearShooterPosition, cmd.endPos)
        }
    }
    fun testRedTowardsGrid(angle: SIUnit<Radian>) {
        testScenario(angle, DriverStation.Alliance.Red, shouldBeFront=true)
    }

    fun testRedAwayFromGrid(angle: SIUnit<Radian>) {
        testScenario(angle, DriverStation.Alliance.Red, shouldBeFront=false)
    }

    fun testBlueTowardsGrid(angle: SIUnit<Radian>) {
        testScenario(angle, DriverStation.Alliance.Blue, shouldBeFront=true)
    }

    fun testBlueAwayFromGrid(angle: SIUnit<Radian>) {
        testScenario(angle, DriverStation.Alliance.Blue, shouldBeFront=false)
    }

    @Test
    fun testRedTowardsGrid() {
        testRedTowardsGrid(0.degrees)
    }

    @Test
    fun testRedAwayFromGrid() {
        testRedAwayFromGrid(180.degrees)
    }

    @Test
    fun testRedMostlyAwayFromGrid() {
        testRedAwayFromGrid(91.degrees)
    }

    @Test
    fun testRedMostlyAwayFromGrid2() {
        testRedAwayFromGrid(269.degrees)
    }

    @Test
    fun testRedMostlyTowardsGrid() {
        testRedTowardsGrid(89.degrees)
    }

    @Test
    fun testRedMostlyTowardsGrid2() {
        testRedTowardsGrid(271.degrees)
    }

    @Test
    fun testRedMostlyTowardsGrid3() {
        testRedTowardsGrid(359.degrees)
    }

    @Test
    fun testRedMostlyTowardsGrid4() {
        testRedTowardsGrid(719.degrees)
    }

    @Test
    fun testBlueTowardsGrid() {
        testBlueTowardsGrid(180.degrees)
    }

    @Test
    fun testBlueAwayFromGrid() {
        testBlueAwayFromGrid(0.degrees)
    }

    @Test
    fun testBlueMostlyAwayFromGrid() {
        testBlueAwayFromGrid(89.degrees)
    }

    @Test
    fun testBlueMostlyAwayFromGrid2() {
        testBlueAwayFromGrid(271.degrees)
    }

    @Test
    fun testBlueMostlyAwayFromGrid3() {
        testBlueAwayFromGrid(359.degrees)
    }

    @Test
    fun testBlueMostlyAwayFromGrid4() {
        testBlueAwayFromGrid((-1).degrees)
    }

    @Test
    fun testBlueMostlyTowardsGrid() {
        testBlueTowardsGrid(91.degrees)
    }

    @Test
    fun testBlueMostlyTowardsGrid2() {
        testBlueTowardsGrid(269.degrees)
    }

    @Test
    fun testBlueMostlyTowardsGrid3() {
        testBlueTowardsGrid((-91).degrees)
    }

}