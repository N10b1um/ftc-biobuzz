package org.firstinspires.ftc.teamcode.config

import com.acmerobotics.dashboard.config.Config

/** Wiring applies at setup; powers and jam thresholds apply each loop. */
@Config
object IntakeConfig {
    @JvmField var MOTOR_NAME = "intakeMotor"
    @JvmField var INVERT_MOTOR = true
    @JvmField var INTAKE_POWER = 1.0
    @JvmField var EJECT_POWER = -1.0
    @JvmField var ENABLE_ANTI_JAM = true
    @JvmField var CURRENT_POLL_INTERVAL_MS = 20L
    @JvmField var JAM_CURRENT_THRESHOLD_AMPS = 7.0
    @JvmField var STALL_VELOCITY_THRESHOLD_TICKS = 80.0
    @JvmField var STARTUP_GRACE_SEC = 0.25
    @JvmField var MIN_JAM_DETECTION_POWER = 0.20
    @JvmField var JAM_DETECTION_DELAY_SEC = 0.25
    @JvmField var ANTI_JAM_REVERSE_TIME_SEC = 0.35
}
