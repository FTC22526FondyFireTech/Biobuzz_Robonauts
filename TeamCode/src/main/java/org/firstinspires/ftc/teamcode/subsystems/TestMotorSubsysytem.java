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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class TestMotorSubsysytem extends SubsystemBase {


    public MotorEx testMotor;

    public PIDController pidController;
    public double targetDegreesPerSec;


    public static boolean changeCatapultPositionCoefficients = false;
    public static double catapultPositionKp = .02;
    public static double catapultPositionKi = 0;
    public static double catapultPositionKd = 0;


    public static double catapultPositionDownKp = .005;
    public static double catapultPositionDownKi = 0;
    public static double catapultPositionDownKd = 0;
    public int test;
    public double output;
    public double cpr;
    public double maxRPM;
    private double upAngle = 90;
    private double downAngle = 0;
    public ElapsedTime stateTimer = new ElapsedTime();
    private double pidOut;

    public TestMotorSubsysytem(HardwareMap hardwareMap) {
        testMotor = new MotorEx(hardwareMap, "testMotor", Motor.GoBILDA.RPM_223);
        cpr = testMotor.getCPR();
        maxRPM = testMotor.getMaxRPM();
        testMotor.setRunMode(Motor.RunMode.RawPower);
        testMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        testMotor.setInverted(false);
        targetDegreesPerSec = 750;
        pidController = new PIDController(catapultPositionKp, catapultPositionKi, catapultPositionKd);
        pidController.reset();
        double maxOut = targetDegreesPerSec / (maxRPM * 360. / 60);
        pidController.setMaxOutput(maxOut);
    }


    @Override
    public void periodic() {
        if (runCatapult)
            runCatapult();

        if (changeCatapultPositionCoefficients) {
            pidController.setPID(catapultPositionKp, catapultPositionKi, catapultPositionKd);
        }
    }

    public double getCatapultAngleDegrees() {
        return testMotor.getCurrentPosition() * 360. / cpr;
    }

    public double getCatapultVelocity() {
        return testMotor.getVelocity() * 360. / cpr;
    }

    public double getMotorAmps() {
        return testMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void positionCatapult(double targetDegrees) {
        test++;
        output = pidController.calculate(getCatapultAngleDegrees());
        testMotor.set(output);
    }

    public Command positionCatapultCommand(double targetDegrees) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    pidController.reset();
                    pidController.setSetPoint(targetDegrees);
                }),
                Commands.run(() -> positionCatapult(targetDegrees)).interruptOn(() -> atGoal())
                        .andThen(Commands.runOnce(() -> testMotor.set(0))));
    }

    public Command cycleCatapultCommand() {
        return Commands.sequence(
                positionCatapultCommand(upAngle),
                Commands.waitMillis(250),
                positionCatapultCommand(downAngle));
    }

    public boolean atGoal() {
        return pidController.getSetPoint() > 75 && getCatapultAngleDegrees() > 90
                || pidController.getSetPoint() < 25 && getCatapultAngleDegrees() < 0;
    }

    public Command jogTestMotorCommand(double pct) {
        return Commands.runOnce(() -> testMotor.set(pct));
    }

    public Command resetEncoderCommand() {
        return Commands.runOnce(() -> testMotor.resetEncoder());
    }

    public boolean power_on;
    public boolean runCatapult;
    public boolean stopCatapult;
    int upperTicks = 200;
    int lowerTicks = 10;
    public static double raiseTimeMs = 500;
    public static double startMs = 50;

    double lowerTimeMs = 800;
    double upperHoldTimeMs = 100;
    double lowerHoldTimeMs = 120;
    public static double raisePct = .6;
    public static double lowerPct = -.5;
    public static double holdPct = -.02;

    int maxTicksReached;
    int minTicksReached;

    public int state = 0;


    public void runCatapult() {
        if (state == 0) {
            testMotor.resetEncoder();
            pidController.setPID(catapultPositionKp, catapultPositionKi, catapultPositionKd);
            stateTimer.reset();
            state = 1;
        } else if (state == 1) {
            pidOut = pidController.calculate(testMotor.getCurrentPosition(), upperTicks);
            testMotor.set(pidOut);
            // testMotor.set(raisePct);
            maxTicksReached = testMotor.getCurrentPosition();
            if (testMotor.getCurrentPosition() > upperTicks) {
                testMotor.set(holdPct);
                state = 2;
                stateTimer.reset();
            }

        } else if (state == 2) {
            testMotor.set(holdPct);
            if (stateTimer.milliseconds() > upperHoldTimeMs) {
                state = 3;
                stateTimer.reset();

                pidController.setPID(catapultPositionDownKp, catapultPositionDownKi, catapultPositionDownKd);
            }

        } else if (state == 3) {
            pidOut = pidController.calculate(testMotor.getCurrentPosition(), lowerTicks);
            testMotor.set(pidOut);
            //  testMotor.set(lowerPct);
            minTicksReached = testMotor.getCurrentPosition();
//            if (testMotor.getCurrentPosition() <= lowerTicks) {
//                testMotor.set(holdPct);
//                state = 4;
//                stateTimer.reset();
//            }

        } else if (state == 4) {
            pidOut = pidController.calculate(testMotor.getCurrentPosition(), lowerTicks);
            testMotor.set(pidOut);
            // testMotor.set(holdPct);
            runCatapult = false;
            stateTimer.reset();
            //  state = 0;
        }


    }


    public void showTelemetry(TelemetryManager telemetryM) {
        telemetryM.addData("PowerOn", power_on);
        telemetryM.addData("RunCatapult", runCatapult);
        telemetryM.addData("State", state);
        telemetryM.addData("PIDOUT", pidOut);
        telemetryM.addData("DrivePct", testMotor.get());
        telemetryM.addData("StateMS", stateTimer.milliseconds());
        telemetryM.addData("PositionTicks", testMotor.getCurrentPosition());
        telemetryM.addData("MaxTicks", maxTicksReached);
        telemetryM.addData("MinTicks", minTicksReached);


    }

}





