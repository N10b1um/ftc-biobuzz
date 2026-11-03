package org.firstinspires.ftc.teamcode.config

import com.acmerobotics.dashboard.config.Config

/** Hardware names and localization settings apply at setup; motor directions and drive tuning apply each loop. */
@Config
object DriveConfig {
    @JvmField var FRONT_LEFT_NAME = "leftFront"
    @JvmField var FRONT_RIGHT_NAME = "rightFront"
    @JvmField var BACK_LEFT_NAME = "leftBack"
    @JvmField var BACK_RIGHT_NAME = "rightBack"
    @JvmField var INVERT_FRONT_LEFT = false
    @JvmField var INVERT_BACK_LEFT = false
    @JvmField var INVERT_FRONT_RIGHT = true
    @JvmField var INVERT_BACK_RIGHT = true

    @JvmField var TRACK_WIDTH_MM = 320.0
    @JvmField var WHEEL_BASE_MM = 320.0
    @JvmField var REFERENCE_EFFECTIVE_RADIUS_MM = 320.0
    @JvmField var LATERAL_MULTIPLIER = 1.15
    @JvmField var ENABLE_VOLTAGE_COMPENSATION = true
    @JvmField var NOMINAL_VOLTAGE = 12.0
    @JvmField var MIN_VALID_VOLTAGE = 8.0
    @JvmField var MIN_VOLTAGE_COMP_SCALE = 0.8
    @JvmField var MAX_VOLTAGE_COMP_SCALE = 1.25

    @JvmField var PINPOINT_NAME = "pinpoint"
    @JvmField var PINPOINT_X_OFFSET_MM = 0.0
    @JvmField var PINPOINT_Y_OFFSET_MM = 0.0
    @JvmField var PINPOINT_TICKS_PER_MM = 13.26291192
    @JvmField var PINPOINT_X_REVERSED = false
    @JvmField var PINPOINT_Y_REVERSED = false
}
