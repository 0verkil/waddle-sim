package com.necessaryevil.simulatedsdk

import com.necessaryevil.simulatedsdk.ftc.SimulatedHardware
import com.necessaryevil.simulatedsdk.ftc.Simulation
import com.necessaryevil.simulatedsdk.ftc.hardware.SimulatedDcMotor
import com.necessaryevil.simulatedsdk.physics.common.SimulatedMotor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import io.kotest.core.spec.style.FunSpec
import org.psilynx.psikit.Logger

@TeleOp
class TestOpMode : OpMode() {

    val testMotor: DcMotor by lazy { hardwareMap.dcMotor.get("test") }

    override fun init() {

        Logger.recordMetadata("opMode name", "Test OpMode")
    }

    override fun loop() {

        testMotor.power = 1.0

        Logger.recordOutput("Motor position (encoder ticks)", testMotor.currentPosition)

    }

}

const val DELTA_TIME = 0.001

class TestSimulator : FunSpec( {

    /*test("simulate") {
        val opmode = TestOpMode()

        // emulate this stuff that doesn't matter
        //val activity = EmulatedFtcRobotControllerActivity()
        val simMotor = SimulatedMotor.GOBILDA_435
        opmode.hardwareMap = HardwareMap(null, null)
        opmode.hardwareMap.dcMotor.put("test",
            DcMotorSimulated(0, simMotor, DcMotorSimulated.MotorType.GOBILDA_435)
        )

        var elapsedTime = 0.0

        Logger.setTimeSource { elapsedTime }

        opmode.init()
        while (true) {
            opmode.loop()
            simMotor.update(DELTA_TIME)

            elapsedTime += DELTA_TIME

            sleep(2)
            Logger.recordOutput("True Motor Angle", simMotor.angle)
            Logger.recordOutput("True Motor Angular Velocity", simMotor.deltaAngle / DELTA_TIME)
        }
    }*/

    test("simulateWithSimulator") {
        SimulatedHardware.addMotor("test", SimulatedMotor.GOBILDA_435, SimulatedDcMotor.MotorType.GOBILDA_435)

        val opmode = TestOpMode()
        Simulation.addSimulation(opmode)

        Simulation.runNextSimulation()

    }

})