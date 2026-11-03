package org.firstinspires.ftc.teamcode.module

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.config.IntakeConfig
import org.firstinspires.ftc.teamcode.config.ModuleConfig
import org.firstinspires.ftc.teamcode.module.Dribbler.DribblerState.*
import org.firstinspires.ftc.teamcode.support.ConfigSnapshot
import org.firstinspires.ftc.teamcode.support.MotorStub
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class DribblerTest {
    private val snapshot = ConfigSnapshot()
    private val motor = MotorStub()
    private var now = 0.0
    private val dribbler = Dribbler(motor.motor) { now }

    @After fun restoreConfig() = snapshot.close()

    @Test fun intakeAppliesRequestedPower() {
        dribbler.setDribblerState(INTAKE, 0.6)
        dribbler.update()
        assertEquals(INTAKE, dribbler.state)
        assertEquals(0.6, motor.power, 0.0)
    }

    @Test fun dropAndStopApplyImmediately() {
        dribbler.setDribblerState(DROP)
        assertEquals(-1.0, motor.power, 0.0)
        dribbler.stop()
        assertEquals(0.0, motor.power, 0.0)
        assertEquals(IDLE, dribbler.state)
    }

    @Test fun jamRequiresStartupGraceAndSustainedStall() {
        motor.velocity = 0.0
        dribbler.setDribblerState(INTAKE)
        dribbler.update()
        now = 0.2
        dribbler.update()
        assertEquals(INTAKE, dribbler.state)
        now = 0.3
        dribbler.update()
        now = 0.5
        dribbler.update()
        assertEquals(INTAKE, dribbler.state)
        now = 0.6
        dribbler.update()
        assertEquals(CLEARING_JAM, dribbler.state)
        assertEquals(-1.0, motor.power, 0.0)
    }

    @Test fun highCurrentTriggersRecoveryEvenWithVelocity() {
        motor.current = 8.0
        startJam()
        assertEquals(CLEARING_JAM, dribbler.state)
    }

    @Test fun repeatedIntakeDoesNotRestartRecoveryTimer() {
        motor.velocity = 0.0
        startJam()
        now = 0.7
        dribbler.setDribblerState(INTAKE, 0.7)
        dribbler.update()
        assertEquals(CLEARING_JAM, dribbler.state)
        now = 1.0
        dribbler.setDribblerState(INTAKE, 0.7)
        dribbler.update()
        assertEquals(INTAKE, dribbler.state)
        assertEquals(0.7, motor.power, 0.0)
    }

    @Test fun releasingTriggerCancelsJamImmediately() {
        motor.velocity = 0.0
        startJam()
        dribbler.setDribblerState(IDLE)
        assertEquals(0.0, motor.power, 0.0)
        now = 2.0
        dribbler.update()
        assertEquals(IDLE, dribbler.state)
        assertEquals(0.0, motor.power, 0.0)
    }

    @Test fun manualDropOverridesJamRecovery() {
        motor.velocity = 0.0
        startJam()
        dribbler.setDribblerState(DROP)
        now = 2.0
        dribbler.update()
        assertEquals(DROP, dribbler.state)
        assertEquals(-1.0, motor.power, 0.0)
    }

    @Test fun sensorRecoveryResetsStallTimer() {
        motor.velocity = 0.0
        dribbler.setDribblerState(INTAKE)
        now = 0.3
        dribbler.update()
        motor.velocity = 1000.0
        now = 0.4
        dribbler.update()
        motor.velocity = 0.0
        now = 0.5
        dribbler.update()
        now = 0.7
        dribbler.update()
        assertEquals(INTAKE, dribbler.state)
    }

    @Test fun disabledAntiJamDoesNotReverseAndResetsPendingStall() {
        motor.velocity = 0.0
        dribbler.setDribblerState(INTAKE)
        now = 0.3
        dribbler.update()
        IntakeConfig.ENABLE_ANTI_JAM = false
        now = 0.6
        dribbler.update()
        IntakeConfig.ENABLE_ANTI_JAM = true
        now = 0.7
        dribbler.update()
        assertEquals(INTAKE, dribbler.state)
    }

    @Test fun sensorReadsAreThrottled() {
        dribbler.update()
        now = 0.01
        dribbler.update()
        assertEquals(1, motor.currentReads)
        now = 0.03
        dribbler.update()
        assertEquals(2, motor.currentReads)
    }

    @Test fun invalidPowerStopsInsteadOfWritingNaN() {
        dribbler.setDribblerState(INTAKE, Double.NaN)
        dribbler.update()
        assertEquals(IDLE, dribbler.state)
        assertEquals(0.0, motor.power, 0.0)
    }

    @Test fun zeroPowerDoesNotStartJamRecovery() {
        motor.velocity = 0.0
        dribbler.setDribblerState(INTAKE, 0.0)
        now = 1.0
        dribbler.update()
        assertEquals(IDLE, dribbler.state)
    }

    @Test fun disabledModuleDoesNotRequireMotor() {
        ModuleConfig.ENABLE_INTAKE = false
        val disabled = Dribbler(HardwareMap(null, null))
        disabled.setup()
        disabled.setDribblerState(INTAKE)
        disabled.update()
        assertEquals(IDLE, disabled.state)
    }

    @Test fun liveIntakePowerLimitsTriggerDemandWithoutRestartingIntake() {
        dribbler.setDribblerState(INTAKE, 0.8)
        dribbler.update()
        assertEquals(0.8, motor.power, 1e-9)
        IntakeConfig.INTAKE_POWER = 0.5
        dribbler.update()
        assertEquals(0.4, motor.power, 1e-9)
        assertEquals(INTAKE, dribbler.state)
    }

    @Test fun invalidConfiguredPowerCannotDriveMotorOrTriggerRecovery() {
        IntakeConfig.INTAKE_POWER = Double.NaN
        motor.velocity = 0.0
        startJam()
        assertEquals(INTAKE, dribbler.state)
        assertEquals(0.0, motor.power, 0.0)
    }

    private fun startJam() {
        dribbler.setDribblerState(INTAKE)
        dribbler.update()
        now = 0.3
        dribbler.update()
        now = 0.6
        dribbler.update()
    }
}
