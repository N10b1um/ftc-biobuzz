package org.firstinspires.ftc.teamcode.module

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.config.IntakeConfig
import org.firstinspires.ftc.teamcode.config.ModuleConfig
import kotlin.math.abs

/** Intake module with sensor caching and nonblocking jam recovery. */
class Dribbler internal constructor(
    private val motor: DcMotorEx?,
    private val seconds: () -> Double = { System.nanoTime() / 1_000_000_000.0 }
) {
    constructor(hardwareMap: HardwareMap) : this(
        if (ModuleConfig.ENABLE_INTAKE) hardwareMap.get(DcMotorEx::class.java, IntakeConfig.MOTOR_NAME) else null
    )

    enum class DribblerState { IDLE, INTAKE, DROP, CLEARING_JAM }

    var state = DribblerState.IDLE
        private set
    var currentAmps = 0.0
        private set
    var velocity = 0.0
        private set
    private var requestedPower = 0.0
    private var stateSince = 0.0
    private var stallSince: Double? = null
    private var lastPoll = Double.NEGATIVE_INFINITY

    fun setup() {
        motor?.apply {
            power = 0.0
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            direction = if (IntakeConfig.INVERT_MOTOR) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        }
        stop()
    }

    /** Repeated intake requests preserve recovery timing; drop and idle interrupt recovery. */
    fun setDribblerState(requested: DribblerState, power: Double = 1.0) {
        when (requested) {
            DribblerState.IDLE -> stop()
            DribblerState.DROP -> {
                stallSince = null
                transition(DribblerState.DROP, seconds())
                motor?.power = ejectPower()
            }
            DribblerState.INTAKE -> {
                requestedPower = if (power.isFinite()) power.coerceIn(0.0, 1.0) else 0.0
                if (requestedPower == 0.0) {
                    stop()
                } else if (state != DribblerState.CLEARING_JAM) {
                    transition(DribblerState.INTAKE, seconds())
                }
            }
            DribblerState.CLEARING_JAM -> error("Jam recovery is controlled by the module")
        }
    }

    fun update() {
        if (motor == null) {
            stop()
            return
        }
        val now = seconds()
        val sampled = now - lastPoll >= IntakeConfig.CURRENT_POLL_INTERVAL_MS.coerceAtLeast(0) / 1000.0
        if (sampled) {
            currentAmps = motor.getCurrent(CurrentUnit.AMPS)
            velocity = motor.velocity
            lastPoll = now
        }
        val intakePower = intakePower()
        when (state) {
            DribblerState.INTAKE -> monitorStall(now, sampled, intakePower)
            DribblerState.CLEARING_JAM -> if (!IntakeConfig.ENABLE_ANTI_JAM ||
                now - stateSince >= safeDuration(IntakeConfig.ANTI_JAM_REVERSE_TIME_SEC)) {
                transition(DribblerState.INTAKE, now)
            }
            else -> stallSince = null
        }
        motor.power = when (state) {
            DribblerState.IDLE -> 0.0
            DribblerState.INTAKE -> intakePower
            DribblerState.DROP, DribblerState.CLEARING_JAM -> ejectPower()
        }
    }

    /** Stops hardware immediately, including during automatic jam recovery. */
    fun stop() {
        state = DribblerState.IDLE
        requestedPower = 0.0
        stallSince = null
        motor?.power = 0.0
    }

    fun log(telemetry: Telemetry) {
        telemetry.addData("Intake enabled", motor != null)
        telemetry.addData("Intake state", state)
        telemetry.addData("Intake current (A)", currentAmps)
        telemetry.addData("Intake velocity (ticks/s)", velocity)
    }

    private fun monitorStall(now: Double, sampled: Boolean, intakePower: Double) {
        if (!IntakeConfig.ENABLE_ANTI_JAM || now - stateSince < safeDuration(IntakeConfig.STARTUP_GRACE_SEC) ||
            intakePower <= 0.0 || intakePower < IntakeConfig.MIN_JAM_DETECTION_POWER) {
            stallSince = null
            return
        }
        if (!sampled) return
        val stalled = (velocity.isFinite() && abs(velocity) < IntakeConfig.STALL_VELOCITY_THRESHOLD_TICKS) ||
            (currentAmps.isFinite() && currentAmps >= IntakeConfig.JAM_CURRENT_THRESHOLD_AMPS)
        if (!stalled) {
            stallSince = null
            return
        }
        val since = stallSince
        if (since == null) {
            stallSince = now
        } else if (now - since >= safeDuration(IntakeConfig.JAM_DETECTION_DELAY_SEC)) {
            transition(DribblerState.CLEARING_JAM, now)
        }
    }

    private fun transition(next: DribblerState, now: Double) {
        if (state == next) return
        state = next
        stateSince = now
        stallSince = null
    }

    private fun intakePower(): Double = IntakeConfig.INTAKE_POWER.let {
        if (it.isFinite()) requestedPower * it.coerceIn(0.0, 1.0) else 0.0
    }

    private fun ejectPower(): Double = IntakeConfig.EJECT_POWER.let {
        if (it.isFinite()) it.coerceIn(-1.0, 0.0) else 0.0
    }

    private fun safeDuration(value: Double): Double = if (value.isFinite()) value.coerceAtLeast(0.0) else 0.0
}
