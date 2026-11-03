package org.firstinspires.ftc.teamcode.module

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.config.DriveConfig

/** Heading source used exclusively by Movement for field-relative driving. */
internal class PinpointHeading(hardwareMap: HardwareMap) {
    private val sensor = hardwareMap.get(GoBildaPinpointDriver::class.java, DriveConfig.PINPOINT_NAME)
    var radians = 0.0
        private set

    fun setup() {
        require(DriveConfig.PINPOINT_X_OFFSET_MM.isFinite())
        require(DriveConfig.PINPOINT_Y_OFFSET_MM.isFinite())
        require(DriveConfig.PINPOINT_TICKS_PER_MM.isFinite() && DriveConfig.PINPOINT_TICKS_PER_MM > 0.0)
        sensor.setOffsets(DriveConfig.PINPOINT_X_OFFSET_MM, DriveConfig.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM)
        sensor.setEncoderResolution(DriveConfig.PINPOINT_TICKS_PER_MM, DistanceUnit.MM)
        sensor.setEncoderDirections(
            direction(DriveConfig.PINPOINT_X_REVERSED), direction(DriveConfig.PINPOINT_Y_REVERSED)
        )
        reset()
    }

    fun update() {
        sensor.update()
        radians = sensor.position.getHeading(AngleUnit.RADIANS)
    }

    fun reset() {
        sensor.resetPosAndIMU()
        radians = 0.0
    }

    private fun direction(reversed: Boolean) = if (reversed) {
        GoBildaPinpointDriver.EncoderDirection.REVERSED
    } else {
        GoBildaPinpointDriver.EncoderDirection.FORWARD
    }
}
