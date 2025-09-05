package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "SparkFun Optical Tracking Odometry Sensor", xmlTag = "SparkFunOTOS")
public class SparkFunOTOS extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // Register addresses for SparkFun OTOS
    private static final int REGISTER_PRODUCT_ID = 0x00;
    private static final int REGISTER_STATUS = 0x01;
    private static final int REGISTER_X_POS_H = 0x03;
    private static final int REGISTER_X_POS_L = 0x04;
    private static final int REGISTER_Y_POS_H = 0x05;
    private static final int REGISTER_Y_POS_L = 0x06;
    private static final int REGISTER_H_POS_H = 0x07;
    private static final int REGISTER_H_POS_L = 0x08;

    public SparkFunOTOS(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x17));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.SparkFun;
    }

    @Override
    public String getDeviceName() {
        return "SparkFun Optical Tracking Odometry Sensor";
    }

    public int getXPosition() {
        byte[] data = deviceClient.read(REGISTER_X_POS_H, 2);
        return (data[0] << 8) | (data[1] & 0xFF);
    }

    public int getYPosition() {
        byte[] data = deviceClient.read(REGISTER_Y_POS_H, 2);
        return (data[0] << 8) | (data[1] & 0xFF);
    }

    public int getHeading() {
        byte[] data = deviceClient.read(REGISTER_H_POS_H, 2);
        return (data[0] << 8) | (data[1] & 0xFF);
    }

    public void resetPosition() {
        // Reset command implementation
        deviceClient.write8(0x02, 0x01);
    }
}