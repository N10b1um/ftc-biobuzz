package org.firstinspires.ftc.teamcode.module

import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.config.DriveConfig
import org.firstinspires.ftc.teamcode.config.ModuleConfig
import org.firstinspires.ftc.teamcode.support.ConfigSnapshot
import org.firstinspires.ftc.teamcode.support.MotorStub
import org.firstinspires.ftc.teamcode.types.MovementParams
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Test

/** X-mecanum model with one external gear mesh and wiring inferred from all three reported motions. */
class MovementWiringTest {
    private val snapshot = ConfigSnapshot()
    private var rightWiresSwapped = false
    private val motors = listOf("leftFront", "rightFront", "leftBack", "rightBack")
        .associateWith { MotorStub() }

    @After fun restoreConfig() = snapshot.close()

    @Test fun previousWiringReproducesAllThreeReportedFailures() {
        rightWiresSwapped = true
        DriveConfig.FRONT_LEFT_NAME = "leftFront"
        DriveConfig.FRONT_RIGHT_NAME = "rightFront"
        DriveConfig.BACK_LEFT_NAME = "leftBack"
        DriveConfig.BACK_RIGHT_NAME = "rightBack"
        DriveConfig.INVERT_FRONT_LEFT = true
        DriveConfig.INVERT_FRONT_RIGHT = true
        DriveConfig.INVERT_BACK_LEFT = true
        DriveConfig.INVERT_BACK_RIGHT = true
        val movement = createMovement()
        movement.moveByParams(MovementParams(0.4, 0.0, 0.0))
        assertMotion(0.0, 0.0, -0.4)
        movement.moveByParams(MovementParams(0.0, 0.4, 0.0))
        assertMotion(0.0, -0.4 * DriveConfig.LATERAL_MULTIPLIER, 0.0)
        movement.moveByParams(MovementParams(0.0, 0.0, 0.4))
        assertMotion(-0.4, 0.0, 0.0)
    }

    @Test fun configuredWiringDrivesForwardAndBackwardWithoutRotation() {
        val movement = createMovement()
        for (power in listOf(-0.4, 0.4)) {
            movement.moveByParams(MovementParams(power, 0.0, 0.0))
            assertMotion(power, 0.0, 0.0)
        }
    }

    @Test fun configuredWiringStrafesBothWaysWithoutRotation() {
        val movement = createMovement()
        for (power in listOf(-0.4, 0.4)) {
            movement.moveByParams(MovementParams(0.0, power, 0.0))
            assertMotion(0.0, power * DriveConfig.LATERAL_MULTIPLIER, 0.0)
        }
    }

    @Test fun configuredWiringTurnsBothWaysWithoutTranslation() {
        val movement = createMovement()
        for (power in listOf(-0.4, 0.4)) {
            movement.moveByParams(MovementParams(0.0, 0.0, power))
            assertMotion(0.0, 0.0, power)
        }
    }

    @Test fun fieldForwardAtQuarterTurnProducesRightwardRobotMotion() {
        val movement = createMovement()
        movement.moveByParams(MovementParams(0.4, 0.0, 0.0).robotRelative(Math.PI / 2.0))
        assertMotion(0.0, 0.4 * DriveConfig.LATERAL_MULTIPLIER, 0.0)
    }

    @Test fun mixedCommandPreservesAllThreeMotionSigns() {
        val movement = createMovement()
        movement.moveByParams(MovementParams(0.2, -0.1, 0.15))
        assertMotion(0.2, -0.1 * DriveConfig.LATERAL_MULTIPLIER, 0.15)
    }

    private fun createMovement(): Movement {
        ModuleConfig.ENABLE_DRIVETRAIN = true
        DriveConfig.ENABLE_VOLTAGE_COMPENSATION = false
        DriveConfig.TRACK_WIDTH_MM = 320.0
        DriveConfig.WHEEL_BASE_MM = 320.0
        DriveConfig.REFERENCE_EFFECTIVE_RADIUS_MM = 320.0
        return Movement({ name -> motors.getValue(name).motor }, { 12.0 }).apply { setup() }
    }

    private fun wheelTravel(name: String, mountingSign: Double): Double {
        val motor = motors.getValue(name)
        val directionSign = if (motor.direction == DcMotorSimple.Direction.REVERSE) -1.0 else 1.0
        val gearMeshSign = -1.0
        return motor.power * directionSign * gearMeshSign * mountingSign
    }

    private fun assertMotion(forward: Double, right: Double, clockwise: Double) {
        val leftFront = wheelTravel("leftFront", -1.0)
        val rightFront = wheelTravel(if (rightWiresSwapped) "rightBack" else "rightFront", 1.0)
        val leftBack = wheelTravel("leftBack", -1.0)
        val rightBack = wheelTravel(if (rightWiresSwapped) "rightFront" else "rightBack", 1.0)
        assertEquals("forward", forward, (leftFront + rightFront + leftBack + rightBack) / 4.0, 1e-9)
        assertEquals("right", right, (leftFront - rightFront - leftBack + rightBack) / 4.0, 1e-9)
        assertEquals("clockwise", clockwise, (leftFront - rightFront + leftBack - rightBack) / 4.0, 1e-9)
    }
}
