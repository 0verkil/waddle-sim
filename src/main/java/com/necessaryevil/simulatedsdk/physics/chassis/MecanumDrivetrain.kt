package com.necessaryevil.simulatedsdk.physics.chassis

import com.necessaryevil.simulatedsdk.physics.common.SimulatedMotor
import com.necessaryevil.simulatedsdk.physics.common.SimulationObject
import org.psilynx.psikit.Logger
import org.psilynx.psikit.wpi.Pose2d
import org.psilynx.psikit.wpi.Rotation2d
import org.psilynx.psikit.wpi.Translation2d
import java.util.function.DoubleSupplier
import kotlin.math.pow
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * Mecanum drivetrain simulation class.
 * @param wheelRadius meters.
 */
class MecanumDrivetrain(
    val mass: Double,
    val moi: Double,
    val wheelRadius: Double,
    val trackwidth: Double,
    val wheelbase: Double,
    val motor: SimulatedMotor,
    val gearRatio: Double = 1.0,
    val efficiency: Double = 0.8,
    val rollingResistance: Double = 0.02,
    val frontalArea: Double = 0.1
) : SimulationObject {

    var pose: Pose2d = Pose2d()
    var velocity: Pose2d = Pose2d()
    
    // Motor power inputs (-1 to 1) as suppliers
    var leftFrontPower: DoubleSupplier = DoubleSupplier { 0.0 }
    var leftBackPower: DoubleSupplier = DoubleSupplier { 0.0 }
    var rightFrontPower: DoubleSupplier = DoubleSupplier { 0.0 }
    var rightBackPower: DoubleSupplier = DoubleSupplier { 0.0 }

    val maxVoltage get() = motor.maxVoltage
    val maxMotorTorque get() = motor.kMotor * (motor.maxVoltage / motor.resistance) * gearRatio
    val motorResistance get() = motor.resistance
    
    // Optional connected motors for automatic power updates
    private var connectedMotors: Array<SimulatedMotor>? = null
    
    // Wheel positions (robot-centric coordinates)
    private val wheelPositions = arrayOf(
        Translation2d(wheelbase/2, trackwidth/2),   // left front
        Translation2d(-wheelbase/2, trackwidth/2),  // left back  
        Translation2d(-wheelbase/2, -trackwidth/2), // right back
        Translation2d(wheelbase/2, -trackwidth/2)   // right front
    )
    
    // Model gripforces (imperfect strafe)
    private val wheelDirections = arrayOf(
        Translation2d(1.0, 0.8).div(hypot(1.0, 0.8)),    // left front
        Translation2d(1.0, -0.8).div(hypot(1.0, 0.8)),   // left back
        Translation2d(1.0, 0.8).div(hypot(1.0, 0.8)),   // right back
        Translation2d(1.0, -0.8).div(hypot(1.0, 0.8))     // right front
    )
    
    private val motorPowers get() = doubleArrayOf(leftFrontPower.asDouble, leftBackPower.asDouble, rightBackPower.asDouble, rightFrontPower.asDouble)
    
    val aerodynamicDrag get() = 0.5 * 1.225 * velocity.translation.norm.pow(2) * frontalArea
    
    override fun update(dt: Double) {
        // Step 1: Compute torques from motor powers
        val wheelTorques = DoubleArray(4)
        for (i in 0..3) {
            val voltage = motorPowers[i] * maxVoltage
            // Proper motor model: T = kt * (V - ke*w) / R * gearRatio
            val wheelAngularVel = getCurrentWheelSpeed(i) // rad/s
            val motorAngularVel = wheelAngularVel * gearRatio // motor shaft speed
            val backEmf = motor.kMotor * motorAngularVel // ke = kt for brushed DC motors
            val current = ((voltage - backEmf) / motorResistance).coerceIn(-20.0, 20.0)  // 20A current limit
            val motorTorque = motor.kMotor * current * efficiency
            wheelTorques[i] = (motorTorque * gearRatio).coerceIn(-maxMotorTorque, maxMotorTorque)
            /*Logger.recordOutput("Debug/Motor $i Current", current)
            Logger.recordOutput("Debug/Motor $i Torque", motorTorque)
            Logger.recordOutput("Debug/$i Back EMF", backEmf)
            Logger.recordOutput("Debug/$i Max Torque", maxMotorTorque)*/
        }
        
        // Step 2: Convert wheel torques to forces
        val wheelForces = DoubleArray(4)
        for (i in 0..3) {
            // Add wheel inertia resistance (wheels need to spin up)
            val wheelInertia = 0.001  // kg⋅m² per wheel
            val wheelAccel = (wheelTorques[i] - wheelForces[i] * wheelRadius * 0.1) / wheelInertia
            // Simplified wheel dynamics - reduces available force during acceleration
            val effectiveTorque = wheelTorques[i] * 0.7  // 70% efficiency during acceleration
            wheelForces[i] = effectiveTorque / wheelRadius
        }
        
        // Step 3: Compute net forces and torques on chassis (all robot-centric)
        var netForceX = 0.0
        var netForceY = 0.0
        var netTorque = 0.0
        
        for (i in 0..3) {
            val force = wheelForces[i]
            val forceVector = wheelDirections[i] * force
            
            netForceX += forceVector.x
            netForceY += forceVector.y
            
            // Torque = r × F (cross product for z-component)
            val r = wheelPositions[i]
            netTorque += r.x * forceVector.y - r.y * forceVector.x
        }
        
        // Step 4: Add resistance forces (in robot frame)
        // Convert field velocity to robot frame for drag calculation
        val robotAngle = pose.rotation
        val robotVelX = robotAngle.cos * velocity.translation.x + robotAngle.sin * velocity.translation.y
        val robotVelY = -robotAngle.sin * velocity.translation.x + robotAngle.cos * velocity.translation.y
        val robotVel = Translation2d(robotVelX, robotVelY)
        
        val dragForce = aerodynamicDrag
        val dragDirection = robotVel.div(robotVel.norm.coerceAtLeast(0.001))
        netForceX -= dragDirection.x * dragForce
        netForceY -= dragDirection.y * dragForce
        
        // Rolling resistance (proportional to normal force)
        val rollingForce = rollingResistance * mass * 9.81
        netForceX -= dragDirection.x * rollingForce  
        netForceY -= dragDirection.y * rollingForce
        
        // Angular drag - INCREASED for better damping
        val angularDrag = velocity.rotation.radians * 0.5  // Increased from 0.1 to 2.0
        netTorque -= angularDrag
        
        // Step 5: Integrate accelerations (all in robot frame)
        val accelX = netForceX / mass
        val accelY = netForceY / mass
        var accelAngular = netTorque / moi

        // Static friction - applies acceleration to bring velocity to zero without overshooting
        val maxStaticAccel = (1.0 / abs(velocity.rotation.radians)).coerceAtMost(10.0)  // Maximum static friction acceleration

        // Calculate acceleration needed to stop in this timestep
        val accelToStop = -velocity.rotation.radians / dt

        // Apply the smaller of: max static friction or what's needed to stop
        val staticFrictionAccel = sign(accelToStop) * minOf(abs(accelToStop), maxStaticAccel)

        accelAngular += staticFrictionAccel

        
        // Update velocities (robot frame)
        val newRobotVelX = robotVelX + accelX * dt
        val newRobotVelY = robotVelY + accelY * dt
        val newVelAngular = velocity.rotation.radians + accelAngular * dt
        
        // Convert robot velocity back to field frame for storage
        val newFieldVelX = pose.rotation.cos * newRobotVelX - pose.rotation.sin * newRobotVelY
        val newFieldVelY = pose.rotation.sin * newRobotVelX + pose.rotation.cos * newRobotVelY
        
        velocity = Pose2d(newFieldVelX, newFieldVelY, Rotation2d(newVelAngular))
        
        // Step 6: Integrate pose
        pose = Pose2d(
            pose.x + velocity.translation.x * dt,
            pose.y + velocity.translation.y * dt,
            pose.rotation + Rotation2d(velocity.rotation.radians * dt)
        )
        
        // Update connected motors automatically
        updateConnectedMotors(dt)
        
        // Logging
        /*Logger.recordOutput("SimpleDrivetrain/Robot Pose", pose)
        Logger.recordOutput("SimpleDrivetrain/Robot Velocity", velocity.translation.norm)
        Logger.recordOutput("SimpleDrivetrain/Robot Angular Velocity", velocity.rotation.degrees)
        Logger.recordOutput("SimpleDrivetrain/Robot Angular Acceleration", accelAngular)
        Logger.recordOutput("SimpleDrivetrain/Net Force X", netForceX)
        Logger.recordOutput("SimpleDrivetrain/Net Force Y", netForceY)
        Logger.recordOutput("SimpleDrivetrain/Net Torque", netTorque)
        
        for (i in 0..3) {
            Logger.recordOutput("SimpleDrivetrain/Wheel $i Torque", wheelTorques[i])
            Logger.recordOutput("SimpleDrivetrain/Wheel $i Speed", getCurrentWheelSpeed(i))
        }*/
    }
    
    private fun getCurrentWheelSpeed(wheelIndex: Int): Double {
        // Compute current wheel speed from chassis motion
        val wheelPos = wheelPositions[wheelIndex]
        val wheelDir = wheelDirections[wheelIndex]
        
        // Convert field velocity to robot frame
        val robotAngle = pose.rotation
        val robotVelX = robotAngle.cos * velocity.translation.x + robotAngle.sin * velocity.translation.y
        val robotVelY = -robotAngle.sin * velocity.translation.x + robotAngle.cos * velocity.translation.y
        val robotVel = Translation2d(robotVelX, robotVelY)
        
        // Translational component (now in robot frame)
        val transComponent = robotVel.x * wheelDir.x + robotVel.y * wheelDir.y
        
        // Rotational component  
        val angularVel = velocity.rotation.radians
        val rotationalLinearVel = angularVel * wheelPos.norm
        val rotationalDirection = Translation2d(-wheelPos.y, wheelPos.x).div(wheelPos.norm.coerceAtLeast(0.001))
        val rotComponent = rotationalLinearVel * (rotationalDirection.x * wheelDir.x + rotationalDirection.y * wheelDir.y)
        
        return (transComponent + rotComponent) / wheelRadius // Convert to angular velocity
    }
    
    /**
     * Set motor powers using standard mecanum drive kinematics
     */
    fun drive(x: Double, y: Double, rotation: Double) {
        val denominator = (abs(y) + abs(x) + abs(rotation)).coerceAtLeast(1.0)
        val lfPower = (y + x + rotation) / denominator
        val lbPower = (y - x + rotation) / denominator  
        val rbPower = (y + x - rotation) / denominator
        val rfPower = (y - x - rotation) / denominator
        
        leftFrontPower = DoubleSupplier { lfPower }
        leftBackPower = DoubleSupplier { lbPower }
        rightBackPower = DoubleSupplier { rbPower }
        rightFrontPower = DoubleSupplier { rfPower }
        
        // Update connected motors if they exist
        connectedMotors?.let { motors ->
            motors[0].power = lfPower  // left front
            motors[1].power = lbPower  // left back
            motors[2].power = rbPower  // right back
            motors[3].power = rfPower  // right front
        }
    }
    
    /**
     * Connect SimulatedMotor instances to automatically receive power updates.
     * This bypasses the motor's physics simulation and uses the drivetrain's unified model instead.
     * 
     * @param leftFront Left front motor
     * @param leftBack Left back motor
     * @param rightBack Right back motor
     * @param rightFront Right front motor
     */
    fun connectMotors(leftFront: SimulatedMotor, leftBack: SimulatedMotor, rightBack: SimulatedMotor, rightFront: SimulatedMotor) {
        connectedMotors = arrayOf(leftFront, leftBack, rightBack, rightFront)
        
        // Set up power suppliers to read from the motors
        leftFrontPower = DoubleSupplier { leftFront.power }
        leftBackPower = DoubleSupplier { leftBack.power }
        rightBackPower = DoubleSupplier { rightBack.power }
        rightFrontPower = DoubleSupplier { rightFront.power }
    }
    
    /**
     * Updates the connected motors with computed wheel speeds, bypassing their internal physics.
     * Call this after update() to sync motor states with drivetrain simulation.
     * 
     * @param dt Time step used in the simulation
     */
    fun updateConnectedMotors(dt: Double) {
        connectedMotors?.let { motors ->
            val wheelSpeeds = getWheelSpeeds()
            for (i in 0..3) {
                // Update motor angular position and velocity to match drivetrain
                motors[i].state.set(0, 0, motors[i].state.get(0, 0) + wheelSpeeds[i] * dt)
                motors[i].state.set(1, 0, wheelSpeeds[i])
                // Keep current unchanged - it's determined by the drivetrain physics
            }
        }
    }
    
    /**
     * Set power suppliers directly for more advanced control
     * 
     * @param leftFront Left front power supplier
     * @param leftBack Left back power supplier
     * @param rightBack Right back power supplier  
     * @param rightFront Right front power supplier
     */
    fun setPowerSuppliers(leftFront: DoubleSupplier, leftBack: DoubleSupplier, rightBack: DoubleSupplier, rightFront: DoubleSupplier) {
        leftFrontPower = leftFront
        leftBackPower = leftBack
        rightBackPower = rightBack
        rightFrontPower = rightFront
    }
    
    /**
     * Get individual wheel speeds for external use
     */
    fun getWheelSpeeds(): DoubleArray {
        return DoubleArray(4) { getCurrentWheelSpeed(it) }
    }
} 