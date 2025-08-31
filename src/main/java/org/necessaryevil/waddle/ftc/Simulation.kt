package org.necessaryevil.waddle.ftc

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import org.psilynx.psikit.Logger
import org.psilynx.psikit.RLOGServer
import java.util.Stack
import kotlin.concurrent.thread

/**
 * Class to handle running simulations.
 * @param opMode For running a standalone opMode.
 * @param stopSeconds The number of seconds to run the standalone OpMode for. Negative seconds denotes running forever.
 * @param deltaMillis Time between simulation steps. Users should not generally modify this.
 */
class Simulation(val opMode: OpMode, val stopSeconds: Double = -1.0, val deltaMillis: Double = 1.0) {

    /**
     * Time since start of simulation, in seconds.
     */
    var currentSeconds = 0.0
    val isStarted get() = SimulatedOpModeManagerImpl.opModeIsStarted as Boolean
    var stopRequested = false

    fun run() {
        // populate hardwaremap and telemetry with emulated instances
        opMode.hardwareMap = SimulatedHardware.hardwareMap
        opMode.telemetry = PsikitTelemetry(opMode)

        // populate gamepads, currently no data updates
        opMode.gamepad1 = Gamepad()
        opMode.gamepad2 = Gamepad()

        // do opmode setup with opmode manager hijack
        SimulatedOpModeManagerImpl.activeOpMode = opMode

        val opModeThread = thread {
            if (opMode is LinearOpMode) {
                opMode.runOpMode()
            } else {
                internalRunOpMode()
            }
        }

        while (opModeThread.isAlive) {
            if (!isStarted) {
                SimulatedOpModeManagerImpl.startOpMode()
            }
            Thread.sleep(10)
        }
    }

    /**
     * Equivalent to hitting the "play" button on your DS: sets `isStarted` to true.
     */
    fun startOpMode() {
        SimulatedOpModeManagerImpl.startOpMode()
    }

    /**
     * Used for running OpModes as opposed to LinearOpModes.
     */
    private fun internalRunOpMode() {
        // If user code in an iterative OpMode throws an exception at any point, no further callback
        // methods will be called because this function will immediately exit.

        // We expect this to take a negligible amount of time,
        // so we need not update the simulation yet. This must actually be true!
        opMode.init()
        update()

        while (!isStarted && !stopRequested) {
            internalPreUserCode()
            opMode.init_loop()
            internalPostUserCode()
            update()
        }

        if (isStarted) {
            // We expect this to take a negligible amount of time,
            // so we need not update the simulation yet. This must actually be true!
            internalPreUserCode()
            opMode.start()
            internalPostUserCode()
            update()

            while (!stopRequested) {
                internalPreUserCode()
                opMode.loop()
                internalPostUserCode()
                update()
            }
        }

        internalPreUserCode()
        opMode.stop()
        internalPostUserCode()
    }

    /**
     * Emulates functionality of a real opmode. Updates simulation time and (TODO) gamepad data.
     */
    private fun internalPreUserCode() {
        Logger.periodicBeforeUser()
        opMode.time = currentSeconds
    }

    /**
     * Emulates functionality of a real opmode. Updates telemetry.
     */
    private fun internalPostUserCode() {
        Logger.periodicAfterUser(0.0, 0.0)
        opMode.telemetry.update()
    }

    /**
     * Handles keeping track of the set of all opmodes queued to run, as opposed to running a single one.
     */
    companion object {
        var isSimulation = false

        val simulations = Stack<Simulation>()
        val simulation: Simulation? get() = simulations.peek()

        val timeoutSeconds get() = simulation?.stopSeconds ?: -1.0
        val time get() = simulation?.currentSeconds ?: 0.0
        val deltaMillis get() = simulation?.deltaMillis ?: 0.001

        var userPeriodic: Runnable = Runnable { }

        var startTime = 0.0

        /**
         * Must be called every loop in a `LinearOpMode`.
         */
        @Synchronized
        fun update() {
            // TODO: make the user code run at the frequency it's supposed to
            if (isSimulation) {

                for (i in 0..5) {
                    SimulatedHardware.update(deltaMillis / 1000.0)

                    simulation?.currentSeconds += deltaMillis / 1000.0

                    if (time > timeoutSeconds && timeoutSeconds > 0) {
                        simulation?.stopRequested = true
                    }
                }

                try {
                    Thread.sleep( (deltaMillis * 10.0).toLong() )
                } catch (e: Exception) {

                }

                userPeriodic.run()

            }
        }

        fun addSimulation(opMode: OpMode, opModeType: OpModeType, deltaMillis: Double = 1.0) {
            when (opModeType) {
                OpModeType.AUTONOMOUS -> addSimulation(opMode, 30.0, deltaMillis)
                OpModeType.TELEOP -> addSimulation(opMode, -1.0, deltaMillis)
                OpModeType.OTHER -> addSimulation(opMode, -1.0, deltaMillis)
            }
        }

        fun addSimulation(opMode: OpMode, stopSeconds: Double = -1.0, deltaMillis: Double = 1.0) {
            this.simulations.add(Simulation(opMode, stopSeconds, deltaMillis))
        }

        fun runNextSimulation(periodic: Runnable = Runnable { }) {
            resetRuntime()
            isSimulation = true
            Logger.setSimulation(true)
            Logger.end()

            // restart logger
            val server = RLOGServer()
            Logger.addDataReceiver(server)

            Logger.setTimeSource { time }

            Logger.start()

            this.userPeriodic = periodic

            simulation?.run()
            simulations.pop()
        }

        /**
         * Emulation of `OpMode.resetRuntime()`.
         */
        fun resetRuntime() {
            startTime = time
        }
    }

    enum class OpModeType {
        AUTONOMOUS,
        TELEOP,
        OTHER
    }

}