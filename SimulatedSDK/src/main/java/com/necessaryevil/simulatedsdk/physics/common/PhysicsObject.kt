package com.necessaryevil.simulatedsdk.physics.common

import org.psilynx.psikit.mechanism.LoggedMechanismLigament2d
import org.psilynx.psikit.mechanism.LoggedMechanismObject2d
import org.psilynx.psikit.wpi.Color8Bit
import org.psilynx.psikit.wpi.Rotation2d
import org.psilynx.psikit.wpi.Translation2d
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

/**
 * Acceleration of gravity.
 */
const val G: Double = 9.81

/**
 * @param mass in Kg.
 */
open class PhysicsLigament(
    name: String,
    val mass: Double,
    length: Double = 0.0,
    angle: Double = 0.0,
    lineWidth: Double = 10.0,
    color: Color8Bit = Color8Bit(235, 137, 52),
    val isCircle: Boolean = false
) : LoggedMechanismLigament2d(name, length, angle, lineWidth, color), SimulationObject {

    init {
        constrainAngleByConstant(angle)
        constrainLengthByConstant(length)
    }

    var angleSupplier: Supplier<Double> = Supplier<Double> { 0.0 }
    var lengthSupplier: Supplier<Double> = Supplier<Double> { 0.0 }

    var deltaAngle: Double = 0.0
    var deltaLength: Double = 0.0

    /**
     * In degrees.
     */
    var angularVelocity: Double = 0.0
    var linearVelocity: Double = 0.0

    val physicsObjects: ArrayList<PhysicsLigament> = ArrayList()
    val centerOfMass: Translation2d
        get() {
            var centerOfMass = Translation2d()

            if (!isCircle) {
                // assume center is the midpoint, weight by mass as a scalar
                centerOfMass =
                    Translation2d(this.length * this.mass / 2.0, Rotation2d.fromDegrees(angle))
            }

            // get the end of the segment
            val ligamentEnd = Translation2d(this.length, Rotation2d.fromDegrees(angle))

            // for every subsegment: append their center of mass, with the origin as the end of the segment and rotated additionally by the segment rotation
            // scale for mass to weight by mass
            for (obj: PhysicsLigament in physicsObjects) {
                centerOfMass.plus(
                    obj.centerOfMass.rotateBy(Rotation2d.fromDegrees(angle)).plus(ligamentEnd)
                        .times(obj.mechanismMass)
                )
            }

            // scale to average center of mass
            centerOfMass
                // divisor: overall mass of all objects
                .div(mechanismMass)
            return centerOfMass
        }

    // array of (moi, mass)
    val moiPoses: ArrayList<Pair<Translation2d, Double>> get() {
        // get position of mois of children
        val out: ArrayList<Pair<Translation2d, Double>> = arrayListOf()

        val ligamentEnd = Translation2d(this.length, Rotation2d.fromDegrees(angle))
        physicsObjects.forEach {
            // for every pose: switch to coordinate frame relative to end of segment
            out.addAll(
                it.moiPoses.map { Pair(it.first.rotateBy(Rotation2d.fromDegrees(angle)).plus(ligamentEnd), it.second) }
            )
        }

        // append our own moi
        if (isCircle) {
            // if it's a circle, moi at (0, 0)
            out.add(Pair(Translation2d(), mass))
        } else {
            // otherwise, moi at end of segment
            out.add(Pair(ligamentEnd, mass))
        }
        return out
    }

    val moi: Double
        get() = moiPoses.fold(0.0) {acc, x -> acc + x.second * x.first.norm.pow(2)}


    val mechanismMass: Double get() {
        var mass = this.mass

        for (obj : PhysicsLigament in physicsObjects) {
            mass += obj.mechanismMass
        }

        return mass
    }

    /**
     * In Nm. Computed based on center of mass.
     */
    open val angularLoad: Double
        get() = centerOfMass.norm * G *
                physicsObjects.fold(this.mass) { acc, x: PhysicsLigament -> acc + x.mass } * cos(
            angle
        )

    open val linearForce: Double
        get() = mass * G * sin(angle)

    open fun constrainAngleByMotor(
        other: SimulatedMotor,
        offsetDegrees: Double = 0.0,
        minDegrees: Double = Double.NEGATIVE_INFINITY,
        maxDegrees: Double = Double.POSITIVE_INFINITY,
        gearRatio: Double = 1.0
    ) {
        this.angle = offsetDegrees
        this.angleSupplier =
            Supplier<Double> { (angle + Math.toDegrees(other.deltaAngle) / gearRatio).coerceIn(minDegrees, maxDegrees) }
        other.addLoad { this.angularLoad }
        other.addMoi { this.moi }
    }

    fun constrainAngleByAngle(
        other: PhysicsLigament,
        offsetDegrees: Double = 0.0,
        minDegrees: Double = Double.NEGATIVE_INFINITY,
        maxDegrees: Double = Double.POSITIVE_INFINITY,
        gearRatio: Double = 1.0
    ) {
        this.angleSupplier =
            Supplier<Double> { (other.angle / gearRatio + offsetDegrees).coerceIn(minDegrees, maxDegrees) }
    }

    fun constrainAngleByDeltaAngle(
        other: PhysicsLigament,
        offsetDegrees: Double = 0.0,
        minDegrees: Double = Double.NEGATIVE_INFINITY,
        maxDegrees: Double = Double.POSITIVE_INFINITY,
        gearRatio: Double = 1.0
    ) {
        this.angle = offsetDegrees
        this.angleSupplier =
            Supplier<Double> { (angle + other.deltaAngle / gearRatio).coerceIn(minDegrees, maxDegrees) }
    }

    fun constrainLengthByLength(
        other: PhysicsLigament,
        offsetMeters: Double = 0.0,
        minMeters: Double = 0.0,
        maxMeters: Double = Double.POSITIVE_INFINITY,
        gearRatio: Double = 1.0
    ) {
        this.lengthSupplier =
            Supplier<Double> { (other.length / gearRatio + offsetMeters).coerceIn(minMeters, maxMeters) }
    }

    fun constrainAngleByLength(
        other: PhysicsLigament,
        offsetDegrees: Double = 0.0,
        minDegrees: Double = Double.NEGATIVE_INFINITY,
        maxDegrees: Double = Double.POSITIVE_INFINITY,
        gearRatio: Double = 1.0
    ) {
        this.angleSupplier = Supplier<Double> {
            ((other.length / this.length).degrees / gearRatio + offsetDegrees).coerceIn(
                minDegrees,
                maxDegrees
            )
        }
    }

    fun constrainLengthByAngle(
        other: PhysicsLigament,
        offsetMeters: Double = 0.0,
        minMeters: Double = 0.0,
        maxMeters: Double = Double.POSITIVE_INFINITY,
        gearRatio: Double = 1.0
    ) {
        this.lengthSupplier = Supplier<Double> {
            (other.angle.radians * other.length / gearRatio + offsetMeters).coerceIn(
                minMeters,
                maxMeters
            )
        }
    }

    fun constrainLengthByDeltaAngle(
        other: PhysicsLigament,
        offsetMeters: Double = 0.0,
        minMeters: Double = 0.0,
        maxMeters: Double = Double.POSITIVE_INFINITY,
        gearRatio: Double = 1.0
    ) {
        this.length = offsetMeters
        this.lengthSupplier = Supplier<Double> {
            (length + other.deltaAngle.radians * other.length / gearRatio).coerceIn(
                minMeters,
                maxMeters
            )
        }
    }

    fun constrainLengthByConstant(meters: Double) {
        this.lengthSupplier = Supplier<Double> { meters }
    }

    fun constrainAngleByConstant(degrees: Double) {
        this.angleSupplier = Supplier<Double> { degrees }
    }

    override fun update(dt: Double) {
        val prevAngle = angle
        val prevLength = angle
        angle = angleSupplier.get()
        length = lengthSupplier.get()
        deltaAngle = angle - prevAngle
        deltaLength = length - prevLength
        angularVelocity = deltaAngle / dt
        linearVelocity = deltaLength / dt
    }

    @Synchronized
    fun <T : LoggedMechanismObject2d> simAppend(obj: T): T {
        if (obj::class == PhysicsLigament::class) {
            physicsObjects.add(obj as PhysicsLigament)
        }
        return append(obj)
    }

}

val Double.degrees: Double get() = this * 180.0 / PI;
val Double.radians: Double get() = this * PI / 180.0;