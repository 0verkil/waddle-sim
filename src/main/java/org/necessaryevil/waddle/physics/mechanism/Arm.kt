package org.necessaryevil.waddle.physics.mechanism

import org.necessaryevil.waddle.physics.common.PhysicsLigament
import org.necessaryevil.waddle.physics.common.SimulatedMotor
import org.necessaryevil.waddle.physics.common.SimulationObject
import org.psilynx.psikit.Logger
import org.psilynx.psikit.mechanism.LoggedMechanism2d
import org.psilynx.psikit.mechanism.LoggedMechanismLigament2d

/**
 * @param x In meters. 0 is the center of the robot, +x is forward, -x is backward.
 * @param gearRatio 19.2 signifies a 19.2:1 reduction; i.e. a larger gear ratio is more torque.
 */
class Arm(
    val name: String,
    x: Double,
    massKilograms: Double,
    lengthInches: Double,
    minDegrees: Double = 0.0,
    maxDegrees: Double = Double.POSITIVE_INFINITY,
    vararg motors: SimulatedMotor,
    gearRatio: Double = 1.0,
    lineWidth: Double = 3.0,
    efficiency: Double = 1.0
) : SimulationObject, Mechanism {

    private val mechanism = LoggedMechanism2d(10.0, 10.0)
    private val root by lazy { mechanism.getRoot("root", 5.0 + x, 0.1) }
    private val arm by lazy {
        root.append(
            PhysicsLigament(
                name,
                massKilograms,
                lengthInches,
                minDegrees,
                lineWidth = lineWidth,
                efficiency = efficiency
            )
        )
    }


    init {
        arm.constrainLengthByConstant(lengthInches * 0.0254)
        arm.constrainAngleByMotors(minDegrees, 0.0, maxDegrees - minDegrees, gearRatio, *motors)
    }

    override fun update(dt: Double) {
        arm.update(dt)

        Logger.recordOutput("Simulation/$name", mechanism)
    }

    override fun append(ligament: LoggedMechanismLigament2d) {
        arm.simAppend(ligament)
    }


}