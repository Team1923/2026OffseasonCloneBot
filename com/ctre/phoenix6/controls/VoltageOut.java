/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.ctre.phoenix6.controls;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.jni.ControlJNI;
import com.ctre.phoenix6.hardware.traits.*;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

/**
 * Request a specified voltage.
 * <p>
 * This control mode will attempt to apply the specified voltage to the motor. If the supply voltage is below
 * the requested voltage, the motor controller will output the supply voltage.
 */
public final class VoltageOut implements ControlRequest, Cloneable {
    /**
     * Voltage to attempt to drive at
     * 
     * <ul>
     *   <li> Units: Volts
     * </ul>
     * 
     */
    public double Output;
    /**
     * Set to true to use FOC commutation (requires Phoenix Pro), which increases
     * peak power by ~15% on supported devices (see {@link SupportsFOC}). Set to
     * false to use trapezoidal commutation.
     * <p>
     * FOC improves motor performance by leveraging torque (current) control. 
     * However, this may be inconvenient for applications that require specifying
     * duty cycle or voltage.  CTR-Electronics has developed a hybrid method that
     * combines the performances gains of FOC while still allowing applications to
     * provide duty cycle or voltage demand.  This not to be confused with simple
     * sinusoidal control or phase voltage control which lacks the performance
     * gains.
     */
    public boolean EnableFOC = true;
    /**
     * Set to true to static-brake the rotor when output is zero (or within
     * deadband).  Set to false to use the NeutralMode configuration setting
     * (default). This flag exists to provide the fundamental behavior of this
     * control when output is zero, which is to provide 0V to the motor.
     */
    public boolean OverrideBrakeDurNeutral = false;
    /**
     * Set to true to force forward limiting.  This allows users to use other limit
     * switch sensors connected to robot controller.  This also allows use of active
     * sensors that require external power.
     */
    public boolean LimitForwardMotion = false;
    /**
     * Set to true to force reverse limiting.  This allows users to use other limit
     * switch sensors connected to robot controller.  This also allows use of active
     * sensors that require external power.
     */
    public boolean LimitReverseMotion = false;
    /**
     * Set to true to ignore hardware limit switches and the LimitForwardMotion and
     * LimitReverseMotion parameters, instead allowing motion.
     * <p>
     * This can be useful on mechanisms such as an intake/feeder, where a limit
     * switch stops motion while intaking but should be ignored when feeding to a
     * shooter.
     * <p>
     * The hardware limit faults and Forward/ReverseLimit signals will still report
     * the values of the limit switches regardless of this parameter.
     */
    public boolean IgnoreHardwareLimits = false;
    /**
     * Set to true to ignore software limits, instead allowing motion.
     * <p>
     * This can be useful when calibrating the zero point of a mechanism such as an
     * elevator.
     * <p>
     * The software limit faults will still report the values of the software limits
     * regardless of this parameter.
     */
    public boolean IgnoreSoftwareLimits = false;
    /**
     * Set to true to delay applying this control request until a timesync boundary
     * (requires Phoenix Pro and CANivore). This eliminates the impact of
     * nondeterministic network delays in exchange for a larger but deterministic
     * control latency.
     * <p>
     * This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
     * Additionally, when this is enabled, the UpdateFreqHz of this request should
     * be set to 0 Hz.
     */
    public boolean UseTimesync = false;

    /**
     * The frequency at which this control will update.
     * This is designated in Hertz, with a minimum of 20 Hz
     * (every 50 ms) and a maximum of 1000 Hz (every 1 ms).
     * Some update frequencies are not supported and will be
     * promoted up to the next highest supported frequency.
     * <p>
     * If this field is set to 0 Hz, the control request will
     * be sent immediately as a one-shot frame. This may be useful
     * for advanced applications that require outputs to be
     * synchronized with data acquisition. In this case, we
     * recommend not exceeding 50 ms between control calls.
     */
    public double UpdateFreqHz = 100;

    /**
     * Request a specified voltage.
     * <p>
     * This control mode will attempt to apply the specified voltage to the motor.
     * If the supply voltage is below the requested voltage, the motor controller
     * will output the supply voltage.
     * 
     * @param Output Voltage to attempt to drive at
     */
    public VoltageOut(double Output) {
        this.Output = Output;
    }

    /**
     * Request a specified voltage.
     * <p>
     * This control mode will attempt to apply the specified voltage to the motor.
     * If the supply voltage is below the requested voltage, the motor controller
     * will output the supply voltage.
     * 
     * @param Output Voltage to attempt to drive at
     */
    public VoltageOut(Voltage Output) {
        this(Output.in(Volts));
    }

    @Override
    public String getName() {
        return "VoltageOut";
    }

    @Override
    public String toString() {
        String ss = "Control: VoltageOut\n";
        ss += "    Output: " + Output + " Volts" + "\n";
        ss += "    EnableFOC: " + EnableFOC + "\n";
        ss += "    OverrideBrakeDurNeutral: " + OverrideBrakeDurNeutral + "\n";
        ss += "    LimitForwardMotion: " + LimitForwardMotion + "\n";
        ss += "    LimitReverseMotion: " + LimitReverseMotion + "\n";
        ss += "    IgnoreHardwareLimits: " + IgnoreHardwareLimits + "\n";
        ss += "    IgnoreSoftwareLimits: " + IgnoreSoftwareLimits + "\n";
        ss += "    UseTimesync: " + UseTimesync + "\n";
        return ss;
    }

    @Override
    public StatusCode sendRequest(CANBus network, int deviceHash) {
        return StatusCode.valueOf(ControlJNI.JNI_RequestControlVoltageOut(
                network.getNameUTF8Bytes(), deviceHash, UpdateFreqHz, Output, EnableFOC, OverrideBrakeDurNeutral, LimitForwardMotion, LimitReverseMotion, IgnoreHardwareLimits, IgnoreSoftwareLimits, UseTimesync));
    }

    /**
     * Gets information about this control request.
     *
     * @return Map of control parameter names and corresponding applied values
     */
    @Override
    public Map<String, String> getControlInfo() {
        var controlInfo = new HashMap<String, String>();
        controlInfo.put("Name", getName());
        controlInfo.put("Output", String.valueOf(this.Output));
        controlInfo.put("EnableFOC", String.valueOf(this.EnableFOC));
        controlInfo.put("OverrideBrakeDurNeutral", String.valueOf(this.OverrideBrakeDurNeutral));
        controlInfo.put("LimitForwardMotion", String.valueOf(this.LimitForwardMotion));
        controlInfo.put("LimitReverseMotion", String.valueOf(this.LimitReverseMotion));
        controlInfo.put("IgnoreHardwareLimits", String.valueOf(this.IgnoreHardwareLimits));
        controlInfo.put("IgnoreSoftwareLimits", String.valueOf(this.IgnoreSoftwareLimits));
        controlInfo.put("UseTimesync", String.valueOf(this.UseTimesync));
        return controlInfo;
    }
    
    /**
     * Modifies this Control Request's Output parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Voltage to attempt to drive at
     * 
     * <ul>
     *   <li> Units: Volts
     * </ul>
     * 
     *
     * @param newOutput Parameter to modify
     * @return Itself
     */
    public VoltageOut withOutput(double newOutput) {
        Output = newOutput;
        return this;
    }
    
    /**
     * Modifies this Control Request's Output parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Voltage to attempt to drive at
     * 
     * <ul>
     *   <li> Units: Volts
     * </ul>
     * 
     *
     * @param newOutput Parameter to modify
     * @return Itself
     */
    public VoltageOut withOutput(Voltage newOutput) {
        Output = newOutput.in(Volts);
        return this;
    }
    
    /**
     * Helper method to get this Control Request's Output parameter converted
     * to a unit type. If not using the Java units library, {@link #Output}
     * can be accessed directly instead.
     * <p>
     * Voltage to attempt to drive at
     * 
     * <ul>
     *   <li> Units: Volts
     * </ul>
     * 
     *
     * @return Output
     */
    public Voltage getOutputMeasure() {
        return Volts.of(Output);
    }
    
    /**
     * Modifies this Control Request's EnableFOC parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Set to true to use FOC commutation (requires Phoenix Pro), which increases
     * peak power by ~15% on supported devices (see {@link SupportsFOC}). Set to
     * false to use trapezoidal commutation.
     * <p>
     * FOC improves motor performance by leveraging torque (current) control. 
     * However, this may be inconvenient for applications that require specifying
     * duty cycle or voltage.  CTR-Electronics has developed a hybrid method that
     * combines the performances gains of FOC while still allowing applications to
     * provide duty cycle or voltage demand.  This not to be confused with simple
     * sinusoidal control or phase voltage control which lacks the performance
     * gains.
     *
     * @param newEnableFOC Parameter to modify
     * @return Itself
     */
    public VoltageOut withEnableFOC(boolean newEnableFOC) {
        EnableFOC = newEnableFOC;
        return this;
    }
    
    /**
     * Modifies this Control Request's OverrideBrakeDurNeutral parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Set to true to static-brake the rotor when output is zero (or within
     * deadband).  Set to false to use the NeutralMode configuration setting
     * (default). This flag exists to provide the fundamental behavior of this
     * control when output is zero, which is to provide 0V to the motor.
     *
     * @param newOverrideBrakeDurNeutral Parameter to modify
     * @return Itself
     */
    public VoltageOut withOverrideBrakeDurNeutral(boolean newOverrideBrakeDurNeutral) {
        OverrideBrakeDurNeutral = newOverrideBrakeDurNeutral;
        return this;
    }
    
    /**
     * Modifies this Control Request's LimitForwardMotion parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Set to true to force forward limiting.  This allows users to use other limit
     * switch sensors connected to robot controller.  This also allows use of active
     * sensors that require external power.
     *
     * @param newLimitForwardMotion Parameter to modify
     * @return Itself
     */
    public VoltageOut withLimitForwardMotion(boolean newLimitForwardMotion) {
        LimitForwardMotion = newLimitForwardMotion;
        return this;
    }
    
    /**
     * Modifies this Control Request's LimitReverseMotion parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Set to true to force reverse limiting.  This allows users to use other limit
     * switch sensors connected to robot controller.  This also allows use of active
     * sensors that require external power.
     *
     * @param newLimitReverseMotion Parameter to modify
     * @return Itself
     */
    public VoltageOut withLimitReverseMotion(boolean newLimitReverseMotion) {
        LimitReverseMotion = newLimitReverseMotion;
        return this;
    }
    
    /**
     * Modifies this Control Request's IgnoreHardwareLimits parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Set to true to ignore hardware limit switches and the LimitForwardMotion and
     * LimitReverseMotion parameters, instead allowing motion.
     * <p>
     * This can be useful on mechanisms such as an intake/feeder, where a limit
     * switch stops motion while intaking but should be ignored when feeding to a
     * shooter.
     * <p>
     * The hardware limit faults and Forward/ReverseLimit signals will still report
     * the values of the limit switches regardless of this parameter.
     *
     * @param newIgnoreHardwareLimits Parameter to modify
     * @return Itself
     */
    public VoltageOut withIgnoreHardwareLimits(boolean newIgnoreHardwareLimits) {
        IgnoreHardwareLimits = newIgnoreHardwareLimits;
        return this;
    }
    
    /**
     * Modifies this Control Request's IgnoreSoftwareLimits parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Set to true to ignore software limits, instead allowing motion.
     * <p>
     * This can be useful when calibrating the zero point of a mechanism such as an
     * elevator.
     * <p>
     * The software limit faults will still report the values of the software limits
     * regardless of this parameter.
     *
     * @param newIgnoreSoftwareLimits Parameter to modify
     * @return Itself
     */
    public VoltageOut withIgnoreSoftwareLimits(boolean newIgnoreSoftwareLimits) {
        IgnoreSoftwareLimits = newIgnoreSoftwareLimits;
        return this;
    }
    
    /**
     * Modifies this Control Request's UseTimesync parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Set to true to delay applying this control request until a timesync boundary
     * (requires Phoenix Pro and CANivore). This eliminates the impact of
     * nondeterministic network delays in exchange for a larger but deterministic
     * control latency.
     * <p>
     * This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
     * Additionally, when this is enabled, the UpdateFreqHz of this request should
     * be set to 0 Hz.
     *
     * @param newUseTimesync Parameter to modify
     * @return Itself
     */
    public VoltageOut withUseTimesync(boolean newUseTimesync) {
        UseTimesync = newUseTimesync;
        return this;
    }

    /**
     * Sets the frequency at which this control will update.
     * This is designated in Hertz, with a minimum of 20 Hz
     * (every 50 ms) and a maximum of 1000 Hz (every 1 ms).
     * Some update frequencies are not supported and will be
     * promoted up to the next highest supported frequency.
     * <p>
     * If this field is set to 0 Hz, the control request will
     * be sent immediately as a one-shot frame. This may be useful
     * for advanced applications that require outputs to be
     * synchronized with data acquisition. In this case, we
     * recommend not exceeding 50 ms between control calls.
     *
     * @param newUpdateFreqHz Parameter to modify
     * @return Itself
     */
    @Override
    public VoltageOut withUpdateFreqHz(double newUpdateFreqHz) {
        UpdateFreqHz = newUpdateFreqHz;
        return this;
    }

    /**
     * Sets the frequency at which this control will update.
     * This is designated in Hertz, with a minimum of 20 Hz
     * (every 50 ms) and a maximum of 1000 Hz (every 1 ms).
     * Some update frequencies are not supported and will be
     * promoted up to the next highest supported frequency.
     * <p>
     * If this field is set to 0 Hz, the control request will
     * be sent immediately as a one-shot frame. This may be useful
     * for advanced applications that require outputs to be
     * synchronized with data acquisition. In this case, we
     * recommend not exceeding 50 ms between control calls.
     *
     * @param newUpdateFreqHz Parameter to modify
     * @return Itself
     */
    @Override
    public VoltageOut withUpdateFreqHz(Frequency newUpdateFreqHz) {
        UpdateFreqHz = newUpdateFreqHz.in(Hertz);
        return this;
    }

    @Override
    public VoltageOut clone() {
        try {
            return (VoltageOut)super.clone();
        } catch (CloneNotSupportedException ex) {
            /* this should never happen */
            throw new RuntimeException(ex);
        }
    }
}

