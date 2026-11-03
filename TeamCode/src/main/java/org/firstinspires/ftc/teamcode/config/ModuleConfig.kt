package org.firstinspires.ftc.teamcode.config

import com.acmerobotics.dashboard.config.Config

/** Hardware switches are read at initialization; restart the OpMode after changing them. */
@Config
object ModuleConfig {
    @JvmField var ENABLE_DRIVETRAIN = true
    @JvmField var ENABLE_PINPOINT = true
    @JvmField var ENABLE_INTAKE = true
}
