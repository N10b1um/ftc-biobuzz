package org.firstinspires.ftc.teamcode.support

import org.firstinspires.ftc.teamcode.config.DriveConfig
import org.firstinspires.ftc.teamcode.config.IntakeConfig
import org.firstinspires.ftc.teamcode.config.ModuleConfig
import java.lang.reflect.Modifier

/** Restores Dashboard globals after each test to avoid order-dependent results. */
class ConfigSnapshot : AutoCloseable {
    private val values = listOf(DriveConfig::class.java, IntakeConfig::class.java, ModuleConfig::class.java)
        .flatMap { it.fields.toList() }
        .filter { Modifier.isStatic(it.modifiers) && !Modifier.isFinal(it.modifiers) }
        .associateWith { it.get(null) }

    override fun close() {
        values.forEach { (field, value) -> field.set(null, value) }
    }
}
