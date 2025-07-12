package com.necessaryevil.simulatedsdk.ftc

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class SimulatableOpMode : OpMode() {
    override fun getRuntime(): Double {
        return Simulation.time
    }

    override fun resetRuntime() {
        Simulation.resetRuntime()
    }
}

abstract class SimulatableLinearOpMode : LinearOpMode() {

    var isStarted = false
    var stopRequested = false

    // TODO: override OpModeManagerImpl so we get access to the functions we need

}