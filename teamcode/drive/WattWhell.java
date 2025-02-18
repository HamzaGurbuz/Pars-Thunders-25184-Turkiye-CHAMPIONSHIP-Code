/**
 * Copyright (c) 2024 WATTOS Company
 *
 * This software has been developed by WATTOS Company specifically for the FIRST Tech Challenge (FTC) competitions.
 * Our goal is to assist young engineers and robotics enthusiasts in developing their skills.
 * The software is openly available for educational and research purposes. We encourage you to use this software
 * in any projects or competitions, and we appreciate your contributions!
 *
 * Please remember to contact us before using it for commercial purposes; this way, we can continue to
 * support the development of the software and foster better communication within our community.
 *
 * When sharing or modifying any part of this software, please be sure to acknowledge the original authors.
 * This is important for promoting collaboration and respect within our community.
 *
 * If you have any questions, feedback, or suggestions regarding its use, please feel free to reach out to us.
 * We wish you success in your competitions and hope to explore more together in the world of robotics!
 *
 * Happy competing!
 */



package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "WATTWHELL", xmlTag = "WATTWHELL")
public class WattWhell extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private int previousPosition = 0;
    private int totalTicks = 0;

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        writeShort(Register.CONFIGURATION, (short) 0x0000);
        return true;
    }

    @Override
    public String getDeviceName() {
        return "WattWhell Encoder Sensor";
    }

    public WattWhell(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x36));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public enum Register {
        FIRST(0),
        CONFIGURATION(0x01),
        MANUFACTURER_ID(0x06),
        DEVICE_ID_REVISION(0x07),
        RESOLUTION(0x08),
        ANGLE(0x0E),
        LAST(RESOLUTION.bVal);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public short getManufacturerIDRaw() {
        return readShort(Register.MANUFACTURER_ID);
    }

    public int getPosition() {
        int currentPosition = TypeConversion.byteArrayToShort(deviceClient.read(Register.ANGLE.bVal, 2)) & 0xFFF;

        if (currentPosition - previousPosition > 2000) {
            totalTicks -= (4096 - currentPosition + previousPosition);
        } else if (previousPosition - currentPosition > 2000) {
            totalTicks += (4096 - previousPosition + currentPosition);
        } else {
            totalTicks += (currentPosition - previousPosition);
        }

        previousPosition = currentPosition;
        return totalTicks;
    }

    public void getget () {
        totalTicks = 0;
    }


    public float getAngleInDegrees() {
        int positionValue = getPosition();
        return (positionValue / 4096.0f) * 360.0f;
    }

    public short getDeviceID() {
        return readShort(Register.DEVICE_ID_REVISION);
    }

    public void resetPosition() {
        totalTicks = 0;
        previousPosition = TypeConversion.byteArrayToShort(deviceClient.read(Register.ANGLE.bVal, 2)) & 0xFFF;
    }

}