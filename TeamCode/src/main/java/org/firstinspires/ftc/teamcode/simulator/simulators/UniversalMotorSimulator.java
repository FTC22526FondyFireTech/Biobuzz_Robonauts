package org.firstinspires.ftc.teamcode.simulator.simulators;

import com.seattlesolvers.solverslib.hardware.motors.Motor;

/**
 * Simulates a single drive motor with realistic physics.
 * <p>
 * Takes power input ([-1, 1]) and generates:
 * - Simulated motor velocity (RPM)
 * - Encoder tick output
 * - Current draw estimation
 * <p>
 * Accounts for:
 * - Motor inertia and acceleration curves
 * - Friction and deceleration
 * - Encoder resolution and wheel diameter
 */
public class UniversalMotorSimulator {

    // Motor characteristics (tuned for REV Robotics motors)
    private static final double MOTOR_INERTIA = 0.002; // kg·m²
    private static final double ACCELERATION_TIME = 0.150; // seconds to reach max power

    // Encoder configuration
    public double targetVelocityRPM;
    private boolean oneShot;
    public double ticksPerSecond;
    private double max_RPM;
    // Motor state
    private double currentVelocityRPM = 0;

    public double acceleration;
    private double currentPower = 0;
    private long totalEncoderTicks = 0;

    public long ticksThisFrame;
    private double cpr;
    private boolean inverted;

    private Motor.GoBILDA motor;
    private double motorToLoadRatio;

    public UniversalMotorSimulator(boolean inverted, double maxRPM, double cpr, double motorLoadRatio) {
        this.motorToLoadRatio = motorLoadRatio;
        this.cpr = cpr;
        this.totalEncoderTicks = 0;
        this.max_RPM = maxRPM;
        this.inverted = inverted;
        this.currentVelocityRPM = 0;
        this.oneShot = false;
    }

    public UniversalMotorSimulator(Motor.GoBILDA motor, boolean inverted, double motorLoadRatio) {
        this.motor = motor;
        this.cpr = motor.getCPR();
        this.max_RPM = motor.getRPM();
        this.motorToLoadRatio = motorLoadRatio;
        this.inverted = inverted;
        this.currentVelocityRPM = 0;
        this.totalEncoderTicks = 0;
        this.oneShot = false;
    }


    /**
     * Update motor simulation for one time step.
     *
     * @param targetPower [-1.0, 1.0] motor power command
     * @param dt          time step in seconds
     */
    public void update(double targetPower, double dt) {
        if (!oneShot) {
            if (inverted) currentPower = -targetPower;
            else currentPower = targetPower;
            oneShot = true;
        }
        // Target velocity based on power
        targetVelocityRPM = currentPower * max_RPM;

        // Simulate acceleration/deceleration with time constant
        // Exponential approach: v(t) = v_target + (v_current - v_target) * e^(-t/tau)
        double tau = ACCELERATION_TIME / 2.2; // time constant (empirical)
        acceleration = (targetVelocityRPM - currentVelocityRPM) / tau * dt;
        currentVelocityRPM += acceleration;

        // Clamp to physical limits
        currentVelocityRPM = Math.max(-max_RPM, Math.min(max_RPM, currentVelocityRPM));

        // Calculate encoder ticks this frame
        // RPM -> ticks per second -> ticks this frame
        ticksPerSecond = (currentVelocityRPM / 60.0) * cpr;
        ticksThisFrame = Math.round(ticksPerSecond * dt);
        totalEncoderTicks += ticksThisFrame;
    }

    /**
     * Set motor power directly (useful for test mode).
     */
    public void setPower(double power) {
        currentPower = Math.max(-1.0, Math.min(1.0, power));
    }

    /**
     * Get current motor velocity in RPM.
     */
    public double getMotorRPM() {
        return currentVelocityRPM;
    }

    public double getLoadRPM() {
        return getMotorRPM() / motorToLoadRatio;
    }

    public double getLoadDegreesPerSec() {
        return getLoadRPM() * 360 / 60.;
    }


    /**
     * Get total encoder ticks accumulated since start.
     */
    public long getEncoderTicks() {
        return totalEncoderTicks;
    }

    /**
     * Get linear distance traveled (inches).
     */
    public double getMotorRevs() {
        return totalEncoderTicks / cpr;
    }

    public double getLoadRevs() {
        return getMotorRevs() / motorToLoadRatio;
    }

    public double getLoadDegrees() {
        return getMotorRevs() * 360.;
    }

    /**
     * Reset encoder to zero.
     */
    public void resetEncoder() {
        totalEncoderTicks = 0;
    }

    /**
     * Get current power command [-1, 1].
     */
    public double getPower() {
        return currentPower;
    }

    public boolean isInverted() {
        return inverted;
    }

    public Motor setInverted(boolean inverted) {
        this.inverted = inverted;
        return null;
    }
}
