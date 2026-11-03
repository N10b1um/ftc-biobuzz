package org.firstinspires.ftc.teamcode.handlers

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.module.Dribbler
import org.firstinspires.ftc.teamcode.module.Movement
import org.firstinspires.ftc.teamcode.module.RobotController
import org.firstinspires.ftc.teamcode.types.MovementParams
import kotlin.math.abs

/** TeleOp facade coordinating the two hardware modules and their shared lifecycle. */
class Actions(hardwareMap: HardwareMap) : AutoCloseable {
    private val robotController = RobotController(hardwareMap)
    private val movement = Movement(hardwareMap)
    private val dribbler = Dribbler(hardwareMap)

    val hasHeading: Boolean get() = movement.hasHeading

    fun setup() {
        robotController.setup()
        movement.setup()
        dribbler.setup()
    }

    /** Clears the bulk cache before polling sensors; call once at the start of each loop. */
    fun update() {
        robotController.clearCache()
        movement.update()
    }

    fun moveByParams(params: MovementParams, fieldCentric: Boolean) {
        movement.moveByParams(if (fieldCentric && hasHeading) params.robotRelative(movement.headingRadians) else params)
    }

    fun brake() = movement.brake()

    fun resetHeading() = movement.resetHeading()

    fun setDribblerState(dribble: Boolean, invert: Boolean, power: Double) {
        val state = when {
            dribble && invert -> Dribbler.DribblerState.DROP
            dribble -> Dribbler.DribblerState.INTAKE
            else -> Dribbler.DribblerState.IDLE
        }
        dribbler.setDribblerState(state, power)
        dribbler.update()
    }

    fun log(telemetry: Telemetry) {
        movement.log(telemetry)
        dribbler.log(telemetry)
    }

    override fun close() {
        try {
            movement.brake()
        } finally {
            try {
                dribbler.stop()
            } finally {
                robotController.close()
            }
        }
    }

    companion object {
        fun getFTCDashboardTelemetry(base: Telemetry): Telemetry =
            MultipleTelemetry(base, FtcDashboard.getInstance().telemetry)

        internal fun shapeAxis(axis: Double, deadzone: Double, cubic: Boolean): Double {
            if (!axis.isFinite() || !deadzone.isFinite() || deadzone !in 0.0..1.0) return 0.0
            val value = axis.coerceIn(-1.0, 1.0)
            if (abs(value) < deadzone) return 0.0
            return if (cubic) value * value * value else value
        }
    }
}
