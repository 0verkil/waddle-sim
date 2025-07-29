package com.necessaryevil.simulatedsdk.ftc

import android.annotation.SuppressLint
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier
import com.qualcomm.robotcore.robocol.TelemetryMessage
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices
import java.lang.reflect.Field
import kotlin.reflect.full.createInstance
import java.lang.reflect.Method

/**
 * Class that allows access to internal OpMode methods (start, stop, run).
 * Simulated OpModeManager that uses reflection to access package-private methods
 * without extending OpModeManagerImpl (which has native library dependencies).
 * ONLY USE THESE METHODS:
 * callActiveOpModeStart()
 * callActiveOpModeStop()
 * callActiveOpModeInit()
 * Nothing else is tested in the slightest.
 * To change the active opmode, simply set the `activeOpMode` property.
 */
@SuppressLint("StaticFieldLeak") // ignore this because there is no actual activity
object SimulatedOpModeManagerImpl {

    var activeOpMode: OpMode? = null
    
    // Cache reflection methods for performance
    private val internalStartMethod: Method? by lazy {
        try {
            OpMode::class.java.superclass.getDeclaredMethod("internalStart").apply {
                isAccessible = true
            }
        } catch (e: NoSuchMethodException) {
            println("Warning: internalStart method not found, falling back to start()")
            null
        }
    }
    
    private val internalStopMethod: Method? by lazy {
        try {
            OpMode::class.java.superclass.getDeclaredMethod("internalStop").apply {
                isAccessible = true
            }
        } catch (e: NoSuchMethodException) {
            println("Warning: internalStop method not found, falling back to stop()")
            null
        }
    }

    private val internalRunMethod: Method? by lazy {
        try {
            OpMode::class.java.getDeclaredMethod("internalRunOpMode").apply {
                isAccessible = true
            }
        } catch (e: NoSuchMethodException) {
            println("Warning: internalRunOpMode() method not found")
            null
        }
    }

    private val opModeIsStartedField: Field? by lazy {
        try {
            OpMode::class.java.superclass.getDeclaredField("isStarted").apply {
                isAccessible = true
            }
        } catch (e: NoSuchFieldException) {
            println("Warning: isStarted field not found")
            null
        }
    }

    val opModeIsStarted get() = opModeIsStartedField?.get(activeOpMode)

    /**
     * This function does nothing so that the default
     * OpModeManagerImpl behavior does not reach any states
     * where it requires additional emulation.
     */
    fun initOpMode(opModeName: String, onlyInitIfDefaultIsRunning: Boolean) {
        return
    }

    /**
     * Calls init() on the active OpMode
     */
    fun callActiveOpModeInit() {
        activeOpMode?.let { opMode ->
            try {
                opMode.init()
            } catch (e: Exception) {
                println("Exception in OpMode init: ${e.message}")
                e.printStackTrace()
            }
        }
    }

    /**
     * Calls internalStart() on the active OpMode using reflection,
     * falls back to start() if internalStart() is not available
     */
    fun callActiveOpModeStart() {
        activeOpMode?.let { opMode ->
            try {
                internalStartMethod?.invoke(opMode)// ?: println("AAAA")//opMode.start()
            } catch (e: Exception) {
                println("Exception in OpMode start: ${e.message}")
                e.printStackTrace()
            }
        }
    }

    /**
     * Calls internalStop() on the active OpMode using reflection,
     * falls back to stop() if internalStop() is not available
     */
    fun callActiveOpModeStop() {
        activeOpMode?.let { opMode ->
            try {
                internalStopMethod?.invoke(opMode) ?: opMode.stop()
            } catch (e: Exception) {
                println("Exception in OpMode stop: ${e.message}")
                e.printStackTrace()
            }
        }
    }

    /**
     * The internalInit() function of the OpMode only starts the OpMode thread and runs it, with no other
     * relevant functionality. Since we are bypassing this functionality, it does not need to be called.
     */
    fun initOpMode() {
        callActiveOpModeInit()
    }

    fun startOpMode() {
        callActiveOpModeStart()
    }

    fun stopOpMode() {
        callActiveOpModeStop()
    }

    fun runOpMode() {
        activeOpMode?.let { opMode ->
            try {
                internalRunMethod?.invoke(opMode)
            } catch (e: Exception) {
                println("Exception in OpMode run: ${e.message}")
                e.printStackTrace()
            }
        }
    }

}

abstract class SimulatableOpMode : OpMode() {
    override fun getRuntime(): Double {
        return Simulation.time
    }

    override fun resetRuntime() {
        Simulation.resetRuntime()
    }
}

abstract class SimulatableLinearOpMode : LinearOpMode() {

    // TODO: override OpModeManagerImpl so we get access to the functions we need

}