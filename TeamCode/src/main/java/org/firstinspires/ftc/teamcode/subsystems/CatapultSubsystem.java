package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;
import java.util.Locale;

@Configurable
public class CatapultSubsystem extends SubsystemBase {
    // 5203 Series Yellow Jacket Planetary Gear Motor (26.9:1 Ratio, 223RPM)
    public static double CATAPULT_UP_POWER = -1.0;
    public static double CATAPULT_DOWN_POWER = 1.0;
    public static double CATAPULT_HOLD_POWER = 0.0;
    public static final long CATAPULT_UP_DURATION = 350L;
    public static final long CATAPULT_DOWN_DURATION = 400L;
    public static final double CATAPULT_UP_POSITION = 90;
    public static final double CATAPULT_DOWN_POSITION = 0;
    public double maxMotorRPM;
    public double countsPerRev;
    public double countsPerDegree;
    public double maxMotorRevsPerSec;

    public double maxMotorDegreesPerSec;

    public double speedFactor = .6;

    public final double motorSprocketTeeth = 36;
    public final double catapultSprocketTeeth = 36;
    /**
     * The sprocket teeth are the same so catapult moves thr same as the motor
     * We can just work in degrees
     *
     *
     */
    public double targetDegreesPerSec;
    public static double catapultPositionKp = 0.051;
    public static double catapultPositionKi = 0;
    public static double catapultPositionKd = 0;

    public static double catapultPositionDownKp = 0.051;
    public static double catapultPositionDownKi = 0;
    public static double catapultPositionDownKd = 0;
    public static boolean changeCatapultPositionCoefficients = false;
    public PIDController pidController;
    double upperHoldTimeMs = 100;
    public static double holdPct = -.02;
    public ElapsedTime stateTimer = new ElapsedTime();
    private double pidOut;
    public MotorEx leftCatapultMotor;
    public MotorEx rightCatapultMotor;
    public MotorGroup catapultMotors;

    private double currentPower;
    private int test;
    private double output;
    private int state;
    public boolean power_on;

    private boolean isRunCatapult() {
        return runCatapult;
    }

    public void setRunCatapult() {
        this.runCatapult = true;
    }
    public void resetRunCatapult() {
        this.runCatapult = false;
    }

    public Command cycleCatapultCommand(){
        return Commands.runOnce(()->setRunCatapult());
    }

    public Command stopCycleCatapultCommand(){
        return Commands.runOnce(()->resetRunCatapult());
    }

    public boolean runCatapult;

    public static double catapultUpDegrees = 90;
    public static double catapultDownDegrees = 0;

    public boolean stopCatapult;
    int upperTicks;
    int lowerTicks ;

    int maxTicksReached;
    int minTicksReached;

    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f2(double v) {
        return String.format(Locale.US, "%.2f", v);
    }

    static String f1(double v) {
        return String.format(Locale.US, "%.1f", v);
    }

    static String f0(double v) {
        return String.format(Locale.US, "%.0f", v);
    }

    public CatapultSubsystem(HardwareMap hardwareMap) {
        // specifying motor allows top rpm and counts per revolution to be read from motor

        leftCatapultMotor = new MotorEx(hardwareMap, "leftCatapult", Motor.GoBILDA.RPM_223);
        leftCatapultMotor.setInverted(true);
        rightCatapultMotor = new MotorEx(hardwareMap, "rightCatapult", Motor.GoBILDA.RPM_223);
        rightCatapultMotor.setInverted(false);
        catapultMotors = new MotorGroup(leftCatapultMotor, rightCatapultMotor);
        maxMotorRPM = leftCatapultMotor.getMaxRPM();
        countsPerRev = leftCatapultMotor.getCPR();
        countsPerDegree = countsPerRev / 360.;
        maxMotorRevsPerSec = maxMotorRPM / 60.;
        maxMotorDegreesPerSec = maxMotorRevsPerSec * 360;
        targetDegreesPerSec = maxMotorDegreesPerSec * speedFactor;
        pidController = new PIDController(catapultPositionKp, catapultPositionKi, catapultPositionKd);

    }


    public Command jogCatapultCommand(double pct) {
        return Commands.runOnce(() -> catapultMotors.set(pct));
    }


    public boolean atGoal() {
        return pidController.getSetPoint() > 75 && getLeftCatapultAngleDegrees() > 90
                || pidController.getSetPoint() < 25 && getLeftCatapultAngleDegrees() < 0;
    }

    public double getLeftCatapultAngleDegrees() {
        List<Double> positions = catapultMotors.getPositions();
        return positions.get(0) / countsPerDegree;
    }

    public double getRightCatapultAngleDegrees() {
        List<Double> positions = catapultMotors.getPositions();
        return positions.get(1) / countsPerDegree;
    }

    public double getLeftCatapultDegreesPerSec() {
        return catapultMotors.getVelocities().get(0) / countsPerDegree;
    }

    public double getRightCatapultMotorDegreesOerSec() {
        return catapultMotors.getVelocities().get(1) / countsPerDegree;
    }

    public double getLeftCatapultCurrent() {
        return leftCatapultMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getRightCatapultCurrent() {
        return leftCatapultMotor.getCurrent(CurrentUnit.AMPS);
    }


    public void fireCatapult() {
        if (state == 0) {
            upperTicks = (int) (catapultUpDegrees / countsPerDegree);
            lowerTicks = (int) (catapultDownDegrees / countsPerDegree);
            catapultMotors.resetEncoder();
            pidController.setPID(catapultPositionKp, catapultPositionKi, catapultPositionKd);
            stateTimer.reset();
            state = 1;

        } else if (state == 1) {
            pidOut = pidController.calculate(catapultMotors.getCurrentPosition(), upperTicks);
            catapultMotors.set(pidOut);
            maxTicksReached = catapultMotors.getCurrentPosition();
            if (catapultMotors.getCurrentPosition() >= upperTicks) {
                catapultMotors.set(holdPct);
                state = 2;
                stateTimer.reset();
            }

        } else if (state == 2) {
            catapultMotors.set(holdPct);
            if (stateTimer.milliseconds() > upperHoldTimeMs) {
                state = 3;
                stateTimer.reset();
                pidController.setPID(catapultPositionDownKp, catapultPositionDownKi, catapultPositionDownKd);
            }

        } else if (state == 3) {
            pidOut = pidController.calculate(catapultMotors.getCurrentPosition(), lowerTicks);
            catapultMotors.set(pidOut);
            minTicksReached = catapultMotors.getCurrentPosition();

        } else if (state == 4) {
            pidOut = pidController.calculate(catapultMotors.getCurrentPosition(), lowerTicks);
            catapultMotors.set(pidOut);
            resetRunCatapult();
            stateTimer.reset();
        }


    }


    @Override
    public void periodic() {

        if (isRunCatapult())
            fireCatapult();

        if (changeCatapultPositionCoefficients) {
            pidController.setPID(catapultPositionKp, catapultPositionKi, catapultPositionKd);
            pidController.setPID(catapultPositionDownKp, catapultPositionDownKi, catapultPositionDownKd);
        }
    }


    public void showTelemetry(TelemetryManager telemetryM) {

        telemetryM.addData("LeftDegrees", f0(getLeftCatapultAngleDegrees()));
        telemetryM.addData("RightDegrees", f0(getRightCatapultAngleDegrees()));

        telemetryM.addData("LeftDegreesPerSec", f0(getLeftCatapultDegreesPerSec()));
        telemetryM.addData("MaxDegreesReached", maxTicksReached/countsPerDegree);
        telemetryM.addData("MinDegreesReached", minTicksReached/countsPerDegree);

        telemetryM.addData("PIDOut", pidOut);

        telemetryM.addData("CountsPerDegree", f0(countsPerDegree));
        telemetryM.addData("Upper Ticks ", upperTicks);
        telemetryM.addData("Lower Ticks ", lowerTicks);


    }
}
