package org.firstinspires.ftc.teamcode.support

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import java.lang.reflect.Proxy

/** Controllable FTC motor interface for exercising production modules on the JVM. */
class MotorStub {
    var power = 0.0
    var current = 0.0
    var velocity = 1000.0
    var currentReads = 0
    var failWrites = false
    var mode: DcMotor.RunMode? = null
    var direction: DcMotorSimple.Direction? = null
    var zeroPowerBehavior: DcMotor.ZeroPowerBehavior? = null

    val motor: DcMotorEx = Proxy.newProxyInstance(
        DcMotorEx::class.java.classLoader, arrayOf(DcMotorEx::class.java)
    ) { _, method, args ->
        when (method.name) {
            "setPower" -> {
                check(!failWrites) { "Motor disconnected" }
                power = args!![0] as Double
                null
            }
            "getPower" -> power
            "getCurrent" -> { currentReads++; current }
            "getVelocity" -> velocity
            "setMode" -> { mode = args!![0] as DcMotor.RunMode; null }
            "setDirection" -> { direction = args!![0] as DcMotorSimple.Direction; null }
            "setZeroPowerBehavior" -> { zeroPowerBehavior = args!![0] as DcMotor.ZeroPowerBehavior; null }
            "toString" -> "MotorStub"
            else -> error("Unexpected motor call: ${method.name}")
        }
    } as DcMotorEx
}
