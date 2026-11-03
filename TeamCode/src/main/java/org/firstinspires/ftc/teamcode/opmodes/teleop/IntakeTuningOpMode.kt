package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.IntakeConfig
import org.firstinspires.ftc.teamcode.module.Dribbler
import org.firstinspires.ftc.teamcode.module.RobotController

/** Isolated intake diagnostics using the same hardware module as competition TeleOp. */
@TeleOp(name = "Intake Diagnostics & Tuning", group = "Tuning")
class IntakeTuningOpMode : LinearOpMode() {
    override fun runOpMode() {
        val controller = RobotController(hardwareMap)
        val dribbler = Dribbler(hardwareMap)
        try {
            controller.setup()
            dribbler.setup()
            var peak = 0.0
            telemetry.addData("Controls", "Right trigger: intake; left trigger: eject; A: reset peak")
            telemetry.addData("Ejection power", IntakeConfig.EJECT_POWER)
            telemetry.update()
            waitForStart()
            while (opModeIsActive()) {
                controller.clearCache()
                val power = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
                val state = when {
                    power > 0.0 -> Dribbler.DribblerState.INTAKE
                    power < 0.0 -> Dribbler.DribblerState.DROP
                    else -> Dribbler.DribblerState.IDLE
                }
                dribbler.setDribblerState(state, power)
                dribbler.update()
                peak = if (gamepad1.a) 0.0 else maxOf(peak, dribbler.currentAmps)
                dribbler.log(telemetry)
                telemetry.addData("Peak current (A)", peak)
                telemetry.update()
                idle()
            }
        } finally {
            try {
                dribbler.stop()
            } finally {
                controller.close()
            }
        }
    }
}
