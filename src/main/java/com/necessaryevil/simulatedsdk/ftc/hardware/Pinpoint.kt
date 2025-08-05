package com.necessaryevil.simulatedsdk.ftc.hardware

import com.necessaryevil.simulatedsdk.physics.chassis.MecanumDrivetrain
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import org.ejml.data.DMatrix1Row
import org.ejml.data.DMatrixRMaj
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.robotcore.external.matrices.ColumnMatrixF
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.psilynx.psikit.wpi.Pose2d
import org.psilynx.psikit.wpi.Rotation2d
import kotlin.random.Random

class SimulatedGoBildaPinpointDriver(val drive: MecanumDrivetrain) : GoBildaPinpointDriver(null, false) {

    init {
        // cursed scope bypass ft. reflection
        GoBildaPinpointDriver::class.java.getDeclaredField("deviceStatus").apply { isAccessible = true }
        GoBildaPinpointDriver::class.java.getDeclaredField("loopTime").apply { isAccessible = true }
        GoBildaPinpointDriver::class.java.getDeclaredField("xEncoderValue").apply { isAccessible = true }
        GoBildaPinpointDriver::class.java.getDeclaredField("yEncoderValue").apply { isAccessible = true }
        GoBildaPinpointDriver::class.java.getDeclaredField("xPosition").apply { isAccessible = true }
        GoBildaPinpointDriver::class.java.getDeclaredField("yPosition").apply { isAccessible = true }
        GoBildaPinpointDriver::class.java.getDeclaredField("hOrientation").apply { isAccessible = true }
        GoBildaPinpointDriver::class.java.getDeclaredField("xVelocity").apply { isAccessible = true }
        GoBildaPinpointDriver::class.java.getDeclaredField("yVelocity").apply { isAccessible = true }
        GoBildaPinpointDriver::class.java.getDeclaredField("hVelocity").apply { isAccessible = true }
    }

    var deviceStatus: Int = 0
        set(v) {field = v; Companion.deviceStatus.set(this, v);}
    var loopTime: Int = 0
        set(v) {field = v; Companion.loopTime.set(this, v);}
    var xEncoderValue: Int = 0
        set(v) {field = v; Companion.xEncoderValue.set(this, v);}
    var yEncoderValue: Int = 0
        set(v) {field = v; Companion.yEncoderValue.set(this, v);}
    var xPosition: Double = 0.0
        set(v) {field = v; Companion.xPosition.set(this, v);}
    var yPosition: Double = 0.0
        set(v) {field = v; Companion.yPosition.set(this, v);}
    var hOrientation: Double = 0.0
        set(v) {field = v; Companion.hOrientation.set(this, v);}
    var xVelocity: Double = 0.0
        set(v) {field = v; Companion.xVelocity.set(this, v);}
    var yVelocity: Double = 0.0
        set(v) {field = v; Companion.yVelocity.set(this, v);}
    var hVelocity: Double = 0.0
        set(v) {field = v; Companion.hVelocity.set(this, v);}

    private var offset: Pose2D = Pose2D(DistanceUnit.MM, 0.0, 0.0, AngleUnit.RADIANS, 0.0)

    override fun update() {
        this.xPosition = DistanceUnit.MM.fromInches(drive.pose.x + offset.getX(DistanceUnit.INCH)) // mm
        this.yPosition = DistanceUnit.MM.fromInches(drive.pose.y + offset.getY(DistanceUnit.INCH)) // mm
        this.hOrientation = drive.pose.rotation.radians + offset.getHeading(AngleUnit.RADIANS) // rad

        this.xVelocity = DistanceUnit.MM.fromInches(drive.velocity.x) // mm / s
        this.yVelocity = DistanceUnit.MM.fromInches(drive.velocity.y) // mm / s
        this.hVelocity = drive.velocity.rotation.radians // rad / s
    }

    override fun update(data: ReadData?) {
        update() // we cope
    }

    override fun setYawScalar(yawScalar: Double) {
        // we cope
    }

    override fun setPosition(pos: Pose2D?) {
        if (pos == null) return
        offset = Pose2D(pos.x - drive.pose.x, pos.y - drive.pose.y, pos.h - drive.pose.rotation.radians)
    }
    fun Pose2D(x: Double, y: Double, h: Double) = Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, h)

    val Pose2D.x get() = this.getX(DistanceUnit.INCH)
    val Pose2D.y get() = this.getY(DistanceUnit.INCH)
    val Pose2D.h get() = this.getHeading(AngleUnit.RADIANS)

    override fun setPosX(
        posX: Double,
        distanceUnit: DistanceUnit?
    ) {
        offset = Pose2D(DistanceUnit.INCH.fromUnit(distanceUnit, posX) - drive.pose.x, offset.y, offset.h)
    }

    override fun setPosY(
        posY: Double,
        distanceUnit: DistanceUnit?
    ) {
        offset = Pose2D(offset.x, DistanceUnit.INCH.fromUnit(distanceUnit, posY) - drive.pose.y, offset.h)
    }

    override fun setHeading(
        heading: Double,
        angleUnit: AngleUnit?
    ) {
        offset = Pose2D(offset.x, offset.y, AngleUnit.RADIANS.fromUnit(angleUnit, heading) - drive.pose.rotation.radians)
    }

    companion object {
        val deviceStatus = GoBildaPinpointDriver::class.java.getDeclaredField("deviceStatus")
        val loopTime = GoBildaPinpointDriver::class.java.getDeclaredField("loopTime")
        val xEncoderValue = GoBildaPinpointDriver::class.java.getDeclaredField("xEncoderValue")
        val yEncoderValue = GoBildaPinpointDriver::class.java.getDeclaredField("yEncoderValue")
        val xPosition = GoBildaPinpointDriver::class.java.getDeclaredField("xPosition")
        val yPosition = GoBildaPinpointDriver::class.java.getDeclaredField("yPosition")
        val hOrientation = GoBildaPinpointDriver::class.java.getDeclaredField("hOrientation")
        val xVelocity = GoBildaPinpointDriver::class.java.getDeclaredField("xVelocity")
        val yVelocity = GoBildaPinpointDriver::class.java.getDeclaredField("yVelocity")
        val hVelocity = GoBildaPinpointDriver::class.java.getDeclaredField("hVelocity")
    }

}