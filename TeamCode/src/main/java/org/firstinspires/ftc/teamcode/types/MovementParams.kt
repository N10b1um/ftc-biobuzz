package org.firstinspires.ftc.teamcode.types

/** Forward and right translation, clockwise turn; heading is counterclockwise radians. */
data class MovementParams(val forward: Double, val strafe: Double, val turn: Double) {
    fun robotRelative(heading: Double): MovementParams = copy(
        forward = forward * kotlin.math.cos(heading) - strafe * kotlin.math.sin(heading),
        strafe = forward * kotlin.math.sin(heading) + strafe * kotlin.math.cos(heading)
    )
}
