package com.necessaryevil.simulatedsdk.ftc.hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxUsbDevice
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.robotcore.eventloop.SyncdDevice
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.LynxModuleDescription
import com.qualcomm.robotcore.hardware.LynxModuleMetaList
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
import com.qualcomm.robotcore.hardware.usb.RobotUsbModule
import com.qualcomm.robotcore.util.SerialNumber
import org.firstinspires.ftc.robotcore.external.Consumer
import org.firstinspires.ftc.robotcore.internal.network.RobotCoreCommandList
import org.firstinspires.ftc.robotcore.internal.ui.ProgressParameters
import java.util.concurrent.TimeUnit

class SimulatedLynxModule(isControlHub: Boolean) : LynxModule(SimulatedLynxUsbDevice(), 0, isControlHub, true) {

    override fun setBulkCachingMode(mode: BulkCachingMode?) {

    }

    override fun clearBulkCache() {

    }

    /**
     * Remove function body to prevent thread from running.
     */
    override fun startExecutor() {

    }

}

private class SimulatedLynxUsbDevice() : LynxUsbDevice {

    /**
     * This is the only method that gets used. The rest are literally unusable.
     */
    override fun registerCallback(
        callback: RobotArmingStateNotifier.Callback?,
        doInitialCallback: Boolean
    ) {

    }

    /**
     * Wait, this one too.
     */
    override fun getSerialNumber(): SerialNumber? {
        return SerialNumber.createFake() // i like how there's a method for this
    }

    override fun getRobotUsbDevice(): RobotUsbDevice? {
        TODO("Not yet implemented")
    }

    override fun isSystemSynthetic(): Boolean {
        TODO("Not yet implemented")
    }

    override fun setSystemSynthetic(systemSynthetic: Boolean) {
        TODO("Not yet implemented")
    }

    override fun failSafe() {
        TODO("Not yet implemented")
    }

    override fun changeModuleAddress(
        module: LynxModule?,
        oldAddress: Int,
        runnable: Runnable?
    ) {
        TODO("Not yet implemented")
    }

    override fun getOrAddModule(moduleDescription: LynxModuleDescription?): LynxModule? {
        TODO("Not yet implemented")
    }

    override fun removeConfiguredModule(module: LynxModule?) {
        TODO("Not yet implemented")
    }

    override fun noteMissingModule(moduleAddress: Int, moduleName: String?) {
        TODO("Not yet implemented")
    }

    override fun performSystemOperationOnParentModule(
        parentAddress: Int,
        operation: Consumer<LynxModule?>?,
        timeout: Int,
        timeoutUnit: TimeUnit?
    ) {
        TODO("Not yet implemented")
    }

    override fun performSystemOperationOnConnectedModule(
        moduleAddress: Int,
        parentAddress: Int,
        operation: Consumer<LynxModule?>?,
        timeout: Int,
        timeoutUnit: TimeUnit?
    ) {
        TODO("Not yet implemented")
    }

    override fun keepConnectedModuleAliveForSystemOperations(
        moduleAddress: Int,
        parentAddress: Int
    ): LynxUsbDevice.SystemOperationHandle? {
        TODO("Not yet implemented")
    }

    override fun discoverModules(checkForImus: Boolean): LynxModuleMetaList? {
        TODO("Not yet implemented")
    }

    override fun acquireNetworkTransmissionLock(message: LynxMessage) {
        TODO("Not yet implemented")
    }

    override fun releaseNetworkTransmissionLock(message: LynxMessage) {
        TODO("Not yet implemented")
    }

    override fun transmit(message: LynxMessage?) {
        TODO("Not yet implemented")
    }

    override fun setupControlHubEmbeddedModule(): Boolean {
        TODO("Not yet implemented")
    }

    override fun getDelegationTarget(): LynxUsbDeviceImpl? {
        TODO("Not yet implemented")
    }

    override fun updateFirmware(
        image: RobotCoreCommandList.FWImage?,
        requestId: String?,
        progressConsumer: Consumer<ProgressParameters?>?
    ): RobotCoreCommandList.LynxFirmwareUpdateResp? {
        TODO("Not yet implemented")
    }

    override fun arm() {
        TODO("Not yet implemented")
    }

    override fun pretend() {
        TODO("Not yet implemented")
    }

    override fun armOrPretend() {
        TODO("Not yet implemented")
    }

    override fun disarm() {
        TODO("Not yet implemented")
    }

    override fun close() {
        TODO("Not yet implemented")
    }

    override fun getArmingState(): RobotArmingStateNotifier.ARMINGSTATE? {
        TODO("Not yet implemented")
    }

    override fun unregisterCallback(callback: RobotArmingStateNotifier.Callback?) {
        TODO("Not yet implemented")
    }

    override fun getGlobalWarning(): String? {
        TODO("Not yet implemented")
    }

    override fun shouldTriggerWarningSound(): Boolean {
        TODO("Not yet implemented")
    }

    override fun suppressGlobalWarning(suppress: Boolean) {
        TODO("Not yet implemented")
    }

    override fun setGlobalWarning(warning: String?) {
        TODO("Not yet implemented")
    }

    override fun clearGlobalWarning() {
        TODO("Not yet implemented")
    }

    override fun lockNetworkLockAcquisitions() {
        TODO("Not yet implemented")
    }

    override fun setThrowOnNetworkLockAcquisition(shouldThrow: Boolean) {
        TODO("Not yet implemented")
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer? {
        TODO("Not yet implemented")
    }

    override fun getDeviceName(): String? {
        TODO("Not yet implemented")
    }

    override fun getConnectionInfo(): String? {
        TODO("Not yet implemented")
    }

    override fun getVersion(): Int {
        TODO("Not yet implemented")
    }

    override fun resetDeviceConfigurationForOpMode() {
        TODO("Not yet implemented")
    }

    override fun getShutdownReason(): SyncdDevice.ShutdownReason? {
        TODO("Not yet implemented")
    }

    override fun setOwner(owner: RobotUsbModule?) {
        TODO("Not yet implemented")
    }

    override fun getOwner(): RobotUsbModule? {
        TODO("Not yet implemented")
    }

    override fun disengage() {
        TODO("Not yet implemented")
    }

    override fun engage() {
        TODO("Not yet implemented")
    }

    override fun isEngaged(): Boolean {
        TODO("Not yet implemented")
    }

}

class SimulatedVoltageSensor : VoltageSensor {

    override fun getVoltage(): Double {
        return 12.5; // i am lazy
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer? {
        TODO("Not yet implemented")
    }

    override fun getDeviceName(): String? {
        TODO("Not yet implemented")
    }

    override fun getConnectionInfo(): String? {
        TODO("Not yet implemented")
    }

    override fun getVersion(): Int {
        TODO("Not yet implemented")
    }

    override fun resetDeviceConfigurationForOpMode() {
        TODO("Not yet implemented")
    }

    override fun close() {
        TODO("Not yet implemented")
    }

}