package org.firstinspires.ftc.teamcode.config

import com.acmerobotics.dashboard.config.Config

/** Driver response settings apply live; bindings are defined in MainTeleOp. */
@Config
object TeleOpConfig {
    @JvmField var STICK_DEADZONE = 0.1
    @JvmField var CUBIC_STICKS = true
    @JvmField var DRIVE_POWER = 1.0
    @JvmField var TRIGGER_THRESHOLD = 0.05
    @JvmField var FIELD_CENTRIC_DEFAULT = true
}
