package com.necessaryevil.waddle.physics.mechanism

import com.necessaryevil.waddle.physics.common.PhysicsLigament
import com.necessaryevil.waddle.physics.common.SimulatedMotor
import com.necessaryevil.waddle.physics.common.SimulationObject
import com.necessaryevil.waddle.physics.common.degrees
import org.psilynx.psikit.Logger
import org.psilynx.psikit.mechanism.LoggedMechanism2d
import org.psilynx.psikit.mechanism.LoggedMechanismLigament2d
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min

/**
 * @param x In meters. 0 is the center of the robot, +x is forward, -x is backward.
 * @param gearRatio 19.2 signifies a 19.2:1 reduction; i.e. a larger gear ratio is more torque.
 */
class ExtensionArm(
    val name: String,
    x: Double,
    massKilograms: Double,
    minInches: Double = 0.0,
    maxInches: Double = Double.POSITIVE_INFINITY,
    minDegrees: Double = 0.0,
    maxDegrees: Double = Double.POSITIVE_INFINITY,
    pivotMotors: Array<out SimulatedMotor>,
    extensionMotors: Array<out SimulatedMotor>,
    spoolRadiusMillimeters: Double,
    extensionGearRatio: Double = 1.0,
    pivotGearRatio: Double = 1.0,
    lineWidth: Double = 3.0,
    efficiency: Double = 1.0
) : SimulationObject, Mechanism {

    private val mechanism = LoggedMechanism2d(10.0, 10.0)
    private val root by lazy { mechanism.getRoot("root", 5.0 + x, 0.1) }
    private val spool by lazy {
        root.append(
            PhysicsLigament(
                "spool",
                0.0,
                spoolRadiusMillimeters / 1000.0,
                lineWidth = 0.0,
                isCircle = true
            )
        )
    }
    private val extension by lazy {
        root.append(
            PhysicsLigament(
                name,
                massKilograms,
                minInches,
                minDegrees,
                lineWidth = lineWidth,
                efficiency = efficiency
            )
        )
    }


    init {
        extension.constrainAngleByMotors(minDegrees, 0.0, maxDegrees - minDegrees, pivotGearRatio, *pivotMotors)

        // find min and max angles
        val minMeters = minInches * 0.0254
        val maxMeters = maxInches * 0.0254

        extension.constrainLengthByAngle(spool, minMeters, minMeters, maxMeters)

        val radiusMeters = spoolRadiusMillimeters / 1000.0

        val minRad = minMeters / radiusMeters // theta = L / R
        val maxRad = maxMeters / radiusMeters

        spool.constrainLengthByConstant(radiusMeters)
        spool.constrainAngleByMotors(0.0, 0.0, maxRad.degrees - minRad.degrees, others=extensionMotors, gearRatio=extensionGearRatio)

        for (motor in extensionMotors) {
            motor.addLoad { extension.linearForce * spool.length / (extensionMotors.size * extensionGearRatio) }
        }
    }

    override fun update(dt: Double) {
        spool.update(dt)
        extension.update(dt)

        Logger.recordOutput("Simulation/$name", mechanism)
    }

    override fun append(ligament: LoggedMechanismLigament2d) {
        extension.simAppend(ligament)
    }


}