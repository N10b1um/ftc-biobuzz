package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.TeleOpConfig
import org.firstinspires.ftc.teamcode.handlers.Actions
import org.firstinspires.ftc.teamcode.types.MovementParams

/** Competition TeleOp using the previous season's initialize, move, update and log structure. */
@TeleOp(name = "Main TeleOp", group = "Competition")
class MainTeleOp : LinearOpMode() {
    private lateinit var actions: Actions
    private var fieldCentric = false
    private var previousBack = false
    private var previousOptions = false

    override fun runOpMode() {
        actions = Actions(hardwareMap)
        try {
            initialize()
            waitForStart()
            while (opModeIsActive()) {
                actions.update()
                updateDriveMode()
                move()
                actions.setDribblerState(
                    gamepad1.left_trigger > TeleOpConfig.TRIGGER_THRESHOLD && !gamepad1.circle,
                    gamepad1.left_bumper,
                    gamepad1.left_trigger.toDouble()
                )
                actions.log(telemetry)
                telemetry.addData("Drive mode", if (fieldCentric) "Field centric" else "Robot centric")
                telemetry.update()
                idle()
            }
        } finally {
            actions.close()
        }
    }

    private fun initialize() {
        telemetry = Actions.getFTCDashboardTelemetry(telemetry)
        actions.setup()
        fieldCentric = TeleOpConfig.FIELD_CENTRIC_DEFAULT && actions.hasHeading
        telemetry.addData("Controls", "Left stick: move; right stick X: turn; left stick button: brake")
        telemetry.addData("Intake", "Left trigger: intake; + left bumper: eject; circle: stop intake")
        telemetry.addData("Heading", "Back: drive mode; Options: reset heading")
        telemetry.addData("Status", "Ready")
        telemetry.update()
    }

    private fun move() {
        if (gamepad1.left_stick_button) {
            actions.brake()
        } else {
            manualMove()
        }
    }

    private fun manualMove() {
        val scale = TeleOpConfig.DRIVE_POWER.let { if (it.isFinite()) it.coerceIn(0.0, 1.0) else 0.0 }
        actions.moveByParams(
            MovementParams(
                axis(-gamepad1.left_stick_y.toDouble()) * scale,
                axis(gamepad1.left_stick_x.toDouble()) * scale,
                axis(gamepad1.right_stick_x.toDouble()) * scale
            ),
            fieldCentric
        )
    }

    private fun updateDriveMode() {
        if (gamepad1.back && !previousBack && actions.hasHeading) fieldCentric = !fieldCentric
        if (gamepad1.options && !previousOptions) actions.resetHeading()
        previousBack = gamepad1.back
        previousOptions = gamepad1.options
    }

    private fun axis(value: Double): Double =
        Actions.shapeAxis(value, TeleOpConfig.STICK_DEADZONE, TeleOpConfig.CUBIC_STICKS)
}
