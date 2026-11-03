package org.firstinspires.ftc.teamcode.module

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

/** Owns REV bulk caching and restores the previous modes when the OpMode exits. */
class RobotController(hardwareMap: HardwareMap) : AutoCloseable {
    private val previousModes = hardwareMap.getAll(LynxModule::class.java)
        .associateWith { it.bulkCachingMode }

    fun setup() {
        previousModes.keys.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
    }

    fun clearCache() {
        previousModes.keys.forEach { it.clearBulkCache() }
    }

    override fun close() {
        previousModes.forEach { (hub, mode) -> hub.bulkCachingMode = mode }
    }
}
