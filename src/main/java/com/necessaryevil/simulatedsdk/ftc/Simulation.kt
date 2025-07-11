package com.necessaryevil.simulatedsdk.ftc

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.util.Queue
import java.util.Stack

/**
 * Class to handle running simulations.
 * @param opMode For running a standalone opMode.
 * @param opModeSequence For running a combined set of an Auto and a TeleOp. Defaults to auto = 30s, teleop = 120s.
 * @param deltaMillis Time between simulation steps. Users should not generally modify this.
 * @param stopSeconds The number of seconds to run the standalone OpMode for. Negative seconds denotes running forever.
 */
class Simulation(opMode: OpMode? = null, opModeSequence: OpModeSequence? = null, val deltaMillis: Double = 1.0, stopSeconds: Double = -1.0) {

    val opModes = Stack<Pair<OpMode, Double>>()
    val currentOpMode get() = opModes.peek().first
    val currentTimeout get() = opModes.peek().second

    init {
        if (opMode != null) {
            opModes.add(Pair(opMode, stopSeconds))
        } else if (opModeSequence != null) {
            opModes.addAll(arrayListOf(Pair(opModeSequence.auto, 30.0), Pair(opModeSequence.teleOp, 120.0)))
        } else {
            error("opMode and opModeSequence cannot both be null.")
        }
    }

    fun runNextOpMode() {

    }

    fun run() {

    }

}

data class OpModeSequence(val auto: OpMode, val teleOp: OpMode)