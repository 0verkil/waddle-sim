package com.necessaryevil.waddle.physics.mechanism

import com.necessaryevil.waddle.physics.common.PhysicsLigament
import com.necessaryevil.waddle.physics.common.SimulatedMotor
import com.necessaryevil.waddle.physics.common.SimulationObject
import com.necessaryevil.waddle.physics.common.degrees
import org.psilynx.psikit.Logger
import org.psilynx.psikit.mechanism.LoggedMechanism2d
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min

/**
 * @param x In meters. 0 is the center of the robot, +x is forward, -x is backward.
 * @param massKilograms In kilograms.
 * @param angleDegrees In degrees. 0 is parallel to the tile floor.
 */
class LinearExtension(
    val name: String,
    x: Double,
    massKilograms: Double,
    minInches: Double = 0.0,
    maxInches: Double = Double.POSITIVE_INFINITY,
    angleDegrees: Double,
    spoolRadiusMillimeters: Double,
    vararg motors: SimulatedMotor,
    lineWidth: Double = 3.0,
    efficiency: Double = 1.0
) : SimulationObject {

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
                angleDegrees,
                lineWidth = lineWidth,
                efficiency = efficiency
            )
        )
    }


    init {
        extension.constrainAngleByConstant(angleDegrees)

        // find min and max angles
        val minMeters = minInches * 0.0254
        val maxMeters = maxInches * 0.0254

        extension.constrainLengthByAngle(spool, minMeters, minMeters, maxMeters)

        val radiusMeters = spoolRadiusMillimeters / 1000.0

        val minRad = minMeters / radiusMeters // theta = L / R
        val maxRad = maxMeters / radiusMeters

        spool.constrainLengthByConstant(radiusMeters)
        spool.constrainAngleByMotors(0.0, 0.0, maxRad.degrees - minRad.degrees, others=motors)

        for (motor in motors) {
            motor.addLoad { extension.linearForce * spool.length / motors.size }
        }
    }

    override fun update(dt: Double) {
        spool.update(dt)
        extension.update(dt)

        Logger.recordOutput("Simulation/$name", mechanism)
    }


}