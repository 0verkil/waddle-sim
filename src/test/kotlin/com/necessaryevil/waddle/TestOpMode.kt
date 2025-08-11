package com.necessaryevil.waddle

import com.necessaryevil.waddle.ftc.SimulatedHardware
import com.necessaryevil.waddle.ftc.Simulation
import com.necessaryevil.waddle.ftc.hardware.SimulatedDcMotor
import com.necessaryevil.waddle.physics.common.SimulatedMotor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.VoltageSensor
import io.kotest.core.spec.style.FunSpec
import org.psilynx.psikit.Logger

@TeleOp
class TestOpMode : OpMode() {

    val testMotor: DcMotor by lazy { hardwareMap.dcMotor.get("test") }

    override fun init() {

        Logger.recordMetadata("opMode name", "Test OpMode")
        for (device in hardwareMap.getAll(VoltageSensor::class.java)) {
            Logger.recordOutput("voltage sensor", device.voltage)
        }
    }

    override fun loop() {

        testMotor.power = 1.0

        Logger.recordOutput("Motor position (encoder ticks)", testMotor.currentPosition)

    }

}

const val DELTA_TIME = 0.001

class TestSimulator : FunSpec( {

    test("simulate") {
        SimulatedHardware.addMotor("test", SimulatedMotor.GOBILDA_435, SimulatedDcMotor.MotorType.GOBILDA_435)

        val opmode = TestOpMode()
        Simulation.addSimulation(opmode)

        Simulation.runNextSimulation()

    }

})