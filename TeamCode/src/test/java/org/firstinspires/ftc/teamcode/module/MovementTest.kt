package org.firstinspires.ftc.teamcode.module

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.config.DriveConfig
import org.firstinspires.ftc.teamcode.config.ModuleConfig
import org.firstinspires.ftc.teamcode.support.ConfigSnapshot
import org.firstinspires.ftc.teamcode.support.MotorStub
import org.firstinspires.ftc.teamcode.types.MovementParams
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class MovementTest {
    private val snapshot = ConfigSnapshot()
    private val motors = List(4) { MotorStub() }
    private val movement = Movement(motors.map { it.motor }, { 12.0 })

    @After fun restoreConfig() = snapshot.close()

    @Test fun forwardCommandsEveryMotor() {
        movement.moveByParams(MovementParams(0.5, 0.0, 0.0))
        assertPowers(0.5, 0.5, 0.5, 0.5)
    }

    @Test fun strafeUsesTestedLateralMultiplier() {
        movement.moveByParams(MovementParams(0.0, 0.5, 0.0))
        assertPowers(0.575, -0.575, -0.575, 0.575)
    }

    @Test fun clockwiseTurnUsesOppositeSides() {
        movement.moveByParams(MovementParams(0.0, 0.0, 0.5))
        assertPowers(0.5, -0.5, 0.5, -0.5)
    }

    @Test fun mixedCommandsPreserveRatiosWhenSaturated() {
        movement.moveByParams(MovementParams(1.0, 1.0, 1.0))
        assertPowers(1.0, -1.15 / 3.15, 0.85 / 3.15, 1.15 / 3.15)
    }

    @Test fun rectangularGeometryScalesTurn() {
        DriveConfig.WHEEL_BASE_MM = 400.0
        DriveConfig.TRACK_WIDTH_MM = 360.0
        movement.moveByParams(MovementParams(0.0, 0.0, 0.5))
        assertPowers(0.59375, -0.59375, 0.59375, -0.59375)
    }

    @Test fun fieldForwardAtCounterclockwiseQuarterTurnStrafesRight() {
        val params = MovementParams(0.5, 0.0, 0.0).robotRelative(Math.PI / 2)
        movement.moveByParams(params)
        assertPowers(0.575, -0.575, -0.575, 0.575)
    }

    @Test fun voltageCompensationIsBoundedAndMissingVoltageIsIgnored() {
        val lowBattery = Movement(motors.map { it.motor }, { 9.0 })
        lowBattery.moveByParams(MovementParams(0.4, 0.0, 0.0))
        assertPowers(0.5, 0.5, 0.5, 0.5)
        val missingBattery = Movement(motors.map { it.motor }, { Double.NaN })
        missingBattery.moveByParams(MovementParams(0.4, 0.0, 0.0))
        assertPowers(0.4, 0.4, 0.4, 0.4)
    }

    @Test fun invalidTuningAndNonfiniteInputStopMotors() {
        movement.moveByParams(MovementParams(Double.NaN, 0.0, 0.0))
        assertPowers(0.0, 0.0, 0.0, 0.0)
        DriveConfig.REFERENCE_EFFECTIVE_RADIUS_MM = 0.0
        movement.moveByParams(MovementParams(1.0, 0.0, 1.0))
        assertPowers(0.0, 0.0, 0.0, 0.0)
    }

    @Test fun invertedVoltageBoundsDoNotThrow() {
        DriveConfig.MIN_VOLTAGE_COMP_SCALE = 2.0
        DriveConfig.MAX_VOLTAGE_COMP_SCALE = 1.0
        movement.moveByParams(MovementParams(0.5, 0.0, 0.0))
        assertPowers(0.5, 0.5, 0.5, 0.5)
    }

    @Test fun setupPreservesPowerControlAndBrakeMode() {
        movement.setup()
        motors.forEach {
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, it.mode)
            assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, it.zeroPowerBehavior)
        }
        assertPowers(0.0, 0.0, 0.0, 0.0)
    }

    @Test fun brakeAttemptsRemainingMotorsAfterDisconnectedMotor() {
        movement.moveByParams(MovementParams(0.5, 0.0, 0.0))
        motors[0].failWrites = true
        assertThrows(IllegalStateException::class.java) { movement.brake() }
        motors.drop(1).forEach { assertEquals(0.0, it.power, 0.0) }
    }

    @Test fun disabledMovementDoesNotRequireHardwareOrPinpoint() {
        ModuleConfig.ENABLE_DRIVETRAIN = false
        ModuleConfig.ENABLE_PINPOINT = true
        val disabled = Movement(HardwareMap(null, null))
        disabled.setup()
        disabled.moveByParams(MovementParams(1.0, 1.0, 1.0))
        disabled.brake()
        assertFalse(disabled.hasHeading)
    }

    @Test fun motorDirectionsFollowTheirPhysicalWheelSlots() {
        DriveConfig.INVERT_FRONT_LEFT = false
        DriveConfig.INVERT_FRONT_RIGHT = true
        DriveConfig.INVERT_BACK_LEFT = true
        DriveConfig.INVERT_BACK_RIGHT = false
        movement.setup()
        assertEquals(DcMotorSimple.Direction.FORWARD, motors[0].direction)
        assertEquals(DcMotorSimple.Direction.REVERSE, motors[1].direction)
        assertEquals(DcMotorSimple.Direction.REVERSE, motors[2].direction)
        assertEquals(DcMotorSimple.Direction.FORWARD, motors[3].direction)
    }

    @Test fun dashboardDirectionChangesApplyWithoutRestartingOpMode() {
        DriveConfig.INVERT_FRONT_LEFT = false
        DriveConfig.INVERT_FRONT_RIGHT = false
        DriveConfig.INVERT_BACK_LEFT = false
        DriveConfig.INVERT_BACK_RIGHT = false
        movement.setup()
        movement.moveByParams(MovementParams(0.5, 0.0, 0.0))
        DriveConfig.INVERT_FRONT_LEFT = true
        DriveConfig.INVERT_BACK_LEFT = true
        movement.moveByParams(MovementParams(0.5, 0.0, 0.0))
        assertEquals(DcMotorSimple.Direction.REVERSE, motors[0].direction)
        assertEquals(DcMotorSimple.Direction.FORWARD, motors[1].direction)
        assertEquals(DcMotorSimple.Direction.REVERSE, motors[2].direction)
        assertEquals(DcMotorSimple.Direction.FORWARD, motors[3].direction)
        assertPowers(0.5, 0.5, 0.5, 0.5)
    }

    private fun assertPowers(vararg expected: Double) {
        expected.forEachIndexed { index, power -> assertEquals(power, motors[index].power, 1e-9) }
    }
}
