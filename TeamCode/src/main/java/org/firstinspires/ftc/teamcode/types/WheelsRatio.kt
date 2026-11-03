package org.firstinspires.ftc.teamcode.types

import kotlin.math.abs

/** Wheel commands in front-left, front-right, back-left, back-right order. */
data class WheelsRatio(
    val leftFront: Double,
    val rightFront: Double,
    val leftBack: Double,
    val rightBack: Double
) {
    fun normalized(scale: Double = 1.0): WheelsRatio {
        val values = listOf(leftFront, rightFront, leftBack, rightBack).map { it * scale }
        if (values.any { !it.isFinite() }) return ZERO
        val divisor = maxOf(1.0, values.maxOf { abs(it) })
        return WheelsRatio(values[0] / divisor, values[1] / divisor, values[2] / divisor, values[3] / divisor)
    }

    companion object {
        val ZERO = WheelsRatio(0.0, 0.0, 0.0, 0.0)
    }
}
