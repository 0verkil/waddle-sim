package com.necessaryevil.simulatedsdk

import com.necessaryevil.simulatedsdk.ftc.hardware.DcMotorWrapper
import com.necessaryevil.simulatedsdk.physics.common.SimulatedMotor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import io.kotest.core.spec.style.FunSpec
import org.psilynx.psikit.Logger
import org.psilynx.psikit.RLOGServer
import java.lang.Thread.sleep

@TeleOp
class TestOpMode : OpMode() {

    val testMotor by lazy { hardwareMap.dcMotor.get("test") }

    override fun init() {
        val server = RLOGServer()
        Logger.addDataReceiver(server)
        Logger.recordMetadata("opMode name", "Test OpMode")
        Logger.start()
        Logger.periodicAfterUser(0.0, 0.0)
        Logger.start()
    }

    override fun loop() {
        Logger.periodicBeforeUser()

        testMotor.power = 1.0

        Logger.recordOutput("Motor position (encoder ticks)", testMotor.currentPosition)

        Logger.periodicAfterUser(0.0, 0.0)
    }

    override fun stop() {
        Logger.end()
    }

}

const val DELTA_TIME = 0.001

class TestSimulator : FunSpec( {

    test("simulate") {
        val opmode = TestOpMode()

        // emulate this stuff that doesn't matter
        //val activity = EmulatedFtcRobotControllerActivity()
        val simMotor = SimulatedMotor.GOBILDA_435
        opmode.hardwareMap = HardwareMap(null, null)
        opmode.hardwareMap.dcMotor.put("test",
            DcMotorWrapper(0, simMotor, DcMotorWrapper.MotorType.GOBILDA_435)
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
    }


})