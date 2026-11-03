package org.firstinspires.ftc.teamcode.module

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.config.DriveConfig
import org.firstinspires.ftc.teamcode.config.ModuleConfig
import org.firstinspires.ftc.teamcode.types.MovementParams
import org.firstinspires.ftc.teamcode.types.WheelsRatio

/** Mecanum hardware module retaining this season's power mixing and compensation. */
class Movement internal constructor(
    private val motors: List<DcMotorEx>,
    private val voltage: () -> Double,
    private val headingSource: PinpointHeading? = null
) {
    internal constructor(
        motorByName: (String) -> DcMotorEx,
        voltage: () -> Double,
        headingSource: PinpointHeading? = null
    ) : this(
        if (ModuleConfig.ENABLE_DRIVETRAIN) listOf(
            DriveConfig.FRONT_LEFT_NAME, DriveConfig.FRONT_RIGHT_NAME,
            DriveConfig.BACK_LEFT_NAME, DriveConfig.BACK_RIGHT_NAME
        ).map(motorByName) else emptyList(),
        voltage,
        headingSource
    )

    constructor(hardwareMap: HardwareMap) : this(
        { hardwareMap.get(DcMotorEx::class.java, it) },
        { hardwareMap.voltageSensor.firstOrNull()?.voltage ?: DriveConfig.NOMINAL_VOLTAGE },
        if (ModuleConfig.ENABLE_DRIVETRAIN && ModuleConfig.ENABLE_PINPOINT) PinpointHeading(hardwareMap) else null
    )

    private var appliedDirections: List<Boolean> = emptyList()

    init {
        require(motors.isEmpty() || motors.size == 4)
    }

    val hasHeading: Boolean get() = headingSource != null
    val headingRadians: Double get() = headingSource?.radians ?: 0.0

    fun setup() {
        motors.forEach { motor ->
            motor.power = 0.0
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        synchronizeDirections()
        headingSource?.setup()
    }

    fun update() {
        headingSource?.update()
    }

    fun resetHeading() {
        headingSource?.reset()
    }

    fun moveByParams(params: MovementParams) {
        if (motors.isEmpty()) return
        synchronizeDirections()
        val ratios = paramsToRatios(params, voltage())
        val powers = listOf(ratios.leftFront, ratios.rightFront, ratios.leftBack, ratios.rightBack)
        motors.forEachIndexed { index, motor -> motor.power = powers[index] }
    }

    private fun synchronizeDirections() {
        val inverted = listOf(
            DriveConfig.INVERT_FRONT_LEFT, DriveConfig.INVERT_FRONT_RIGHT,
            DriveConfig.INVERT_BACK_LEFT, DriveConfig.INVERT_BACK_RIGHT
        )
        if (inverted == appliedDirections) return
        brake()
        motors.forEachIndexed { index, motor ->
            motor.direction = if (inverted[index]) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        }
        appliedDirections = inverted
    }

    fun brake() {
        var failure: RuntimeException? = null
        motors.forEach { motor ->
            try {
                motor.power = 0.0
            } catch (exception: RuntimeException) {
                val previous = failure
                if (previous == null) failure = exception else previous.addSuppressed(exception)
            }
        }
        failure?.let { throw it }
    }

    fun log(telemetry: Telemetry) {
        telemetry.addData("Movement enabled", motors.isNotEmpty())
        telemetry.addData("Heading (deg)", Math.toDegrees(headingRadians))
    }

    companion object {
        /** Returns zero commands for invalid live tuning instead of sending NaN to hardware. */
        internal fun paramsToRatios(params: MovementParams, batteryVoltage: Double): WheelsRatio {
            val tuning = listOf(
                params.forward, params.strafe, params.turn, DriveConfig.LATERAL_MULTIPLIER,
                DriveConfig.TRACK_WIDTH_MM, DriveConfig.WHEEL_BASE_MM,
                DriveConfig.REFERENCE_EFFECTIVE_RADIUS_MM
            )
            if (tuning.any { !it.isFinite() } || DriveConfig.LATERAL_MULTIPLIER < 0.0 ||
                DriveConfig.TRACK_WIDTH_MM <= 0.0 || DriveConfig.WHEEL_BASE_MM <= 0.0 ||
                DriveConfig.REFERENCE_EFFECTIVE_RADIUS_MM <= 0.0) return WheelsRatio.ZERO

            val strafe = params.strafe * DriveConfig.LATERAL_MULTIPLIER
            val turn = params.turn * (DriveConfig.WHEEL_BASE_MM + DriveConfig.TRACK_WIDTH_MM) /
                (2.0 * DriveConfig.REFERENCE_EFFECTIVE_RADIUS_MM)
            val ratios = WheelsRatio(
                params.forward + strafe + turn, params.forward - strafe - turn,
                params.forward - strafe + turn, params.forward + strafe - turn
            )
            return ratios.normalized(voltageScale(batteryVoltage))
        }

        private fun voltageScale(voltage: Double): Double {
            if (!DriveConfig.ENABLE_VOLTAGE_COMPENSATION || !voltage.isFinite() ||
                voltage <= DriveConfig.MIN_VALID_VOLTAGE) return 1.0
            val nominal = DriveConfig.NOMINAL_VOLTAGE
            val minimum = DriveConfig.MIN_VOLTAGE_COMP_SCALE
            val maximum = DriveConfig.MAX_VOLTAGE_COMP_SCALE
            if (!nominal.isFinite() || nominal <= 0.0 || !minimum.isFinite() ||
                !maximum.isFinite() || minimum <= 0.0 || maximum < minimum) return 1.0
            return (nominal / voltage).coerceIn(minimum, maximum)
        }
    }
}
