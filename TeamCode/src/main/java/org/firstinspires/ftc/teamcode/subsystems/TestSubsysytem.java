package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.simulator.simulators.UniversalMotorSimulator;

@Configurable
public class TestSubsysytem extends SubsystemBase {


    public UniversalMotorSimulator leftMotorSim;

    public PIDController pidController;
    public double targetDegreesPerSec;


    public static boolean changeCatapultPositionCoefficients = false;
    public static double catapultPositionKp = 0.009;
    public static double catapultPositionKi = 0;
    public static double catapultPositionKd = 0;
    public int test;
    public double output;


    public TestSubsysytem() {
        leftMotorSim = new UniversalMotorSimulator(Motor.GoBILDA.RPM_435, false, 1);
        targetDegreesPerSec = 750;
        pidController = new PIDController(catapultPositionKp, catapultPositionKi, catapultPositionKd);
        pidController.reset();
        pidController.setMaxOutput(.5);
    }


    @Override
    public void periodic() {
        if (changeCatapultPositionCoefficients) {
            pidController.setPID(catapultPositionKp, catapultPositionKi, catapultPositionKd);
        }
    }

    public void positionCatapult(double targetDegrees) {
        test++;
        output = pidController.calculate(leftMotorSim.getLoadDegrees());
        leftMotorSim.setPower(output);
    }

    public Command positionCatapultCommand(double targetDegrees) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    pidController.reset();
                    pidController.setSetPoint(targetDegrees);
                }),
                Commands.run(() -> positionCatapult(targetDegrees)).interruptOn(() -> atGoal()));
    }

    public boolean atGoal() {
        return pidController.getSetPoint() > 75 && leftMotorSim.getLoadDegrees() > 90
                || pidController.getSetPoint() < 25 && leftMotorSim.getLoadDegrees() < 0;
    }

    public Command jogSimCatapultCommand(double pct) {
        return Commands.runOnce(() -> leftMotorSim.setPower(pct));
    }

    public Command resetSimEncoderCommand() {
        return Commands.runOnce(() -> leftMotorSim.resetEncoder());
    }


}




