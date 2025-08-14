package com.necessaryevil.waddle

import com.necessaryevil.waddle.ftc.SimulatableLinearOpMode
import com.necessaryevil.waddle.ftc.SimulatedHardware
import com.necessaryevil.waddle.ftc.Simulation
import com.necessaryevil.waddle.ftc.hardware.SimulatedDcMotor
import com.necessaryevil.waddle.ftc.time.SimulatableElapsedTime
import com.necessaryevil.waddle.physics.common.SimulatedMotor
import com.necessaryevil.waddle.physics.mechanism.Arm
import com.necessaryevil.waddle.physics.mechanism.ExtensionArm
import com.necessaryevil.waddle.physics.mechanism.LinearExtension
import com.qualcomm.robotcore.hardware.DcMotor
import io.kotest.core.spec.style.FunSpec
import org.psilynx.psikit.Logger
import org.psilynx.psikit.wpi.Pose2d
import org.psilynx.psikit.wpi.Rotation2d


class MechanismDemo : SimulatableLinearOpMode() {

    val leftSlide: DcMotor by lazy { hardwareMap.get(DcMotor::class.java, "leftSlide") }
    val rightSlide: DcMotor by lazy { hardwareMap.dcMotor.get("rightSlide") }

    val slidePID: PID = PID(0.0025, 0.0, 0.00001)

    val targetPosition: Double = 1000.0

    override fun runOpMode() {
        leftSlide
        rightSlide

        waitForStart()

        while (opModeIsActive()) {

            Logger.periodicBeforeUser()

            val error = targetPosition - leftSlide.currentPosition.toDouble()

            val power = slidePID.update(error)

            Logger.recordOutput("Power", power)
            Logger.recordOutput("Position", leftSlide.currentPosition)
            Logger.recordOutput("Target", targetPosition)
            Logger.recordOutput("Error", error)

            leftSlide.power = power
            rightSlide.power = 1.0

            Logger.recordOutput("Time", runtime)

            Logger.periodicAfterUser(0.0, 0.0)

            Simulation.update()
        }


    }

}

class MechanismSimulator : FunSpec({

    test("Mechanism Demo") {
        // add both slide motors
        val leftSlide = SimulatedHardware.addMotor(
            "leftSlide",
            SimulatedMotor.GOBILDA_435,
            SimulatedDcMotor.MotorType.GOBILDA_435
        )
        val rightSlide = SimulatedHardware.addMotor(
            "rightSlide",
            SimulatedMotor.GOBILDA_435,
            SimulatedDcMotor.MotorType.GOBILDA_435
        )

        // add a linear extension with spool radius 25mm
        SimulatedHardware.addSimulationObject(LinearExtension("Slides", -0.15, 1.0, 11.0, 40.0, 0.0, 25.0, leftSlide, rightSlide))

        Simulation.addSimulation(MechanismDemo(), 120.0)

        Simulation.runNextSimulation {
            Logger.recordOutput(
                "Simulation/Pose", Pose2d(
                    72.0 * 0.0254, 72.0 * 0.0254,
                    Rotation2d()
                )
            ); Logger.recordOutput("Simulation/Left", leftSlide.state.get(2))
        }

    }

})