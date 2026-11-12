package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.simulator.simulators.MotorSimulator;
import org.firstinspires.ftc.teamcode.utils.Configurables;

@Configurable
public class IntakeSubsystem extends SubsystemBase {
    public static double upPosition = 0;
    public static double noNectarPosition = .3;
    public static double downPosition = .5;

    public static double rightTrimValue;//adjust for sync arm servos
    public Motor intakeMotor;
    private MotorSimulator intakeMotorSim;
    private boolean direction;


    public int pollenCount;
    public int redNectarCount;
    public int blueNectarCount;
    public int allianceNectarCount;
    public int opponemtNectarCount;


    private Servo leftArmServo;
    private Servo rightArmServo;
    public static double intakeSpeed = 0.9; // 1150 RPM * 0.77 (or / 77%) = 885.5
    protected double simDt = 0.01; // 10ms fixed physics step (100Hz)
    private double leftArmTargetPosition;
    private double rightArmTargetPosition;

    public boolean intakeRunning;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        if (!Configurables.doSimulation) {
            intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
            intakeMotor.setInverted(true); // change to true if initially out-taking
            direction = intakeMotor.getInverted();
            leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
            leftArmServo.setDirection(Servo.Direction.REVERSE);
            leftArmServo.setPosition(upPosition);
            rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
            rightArmServo.setDirection(Servo.Direction.FORWARD);
            rightArmServo.setPosition(upPosition + rightTrimValue);

        } else {
            intakeMotorSim = new MotorSimulator(true, 1150);
            intakeMotorSim.setInverted(true);
            direction = intakeMotorSim.isInverted();
        }
        intakeRunning = false;
    }

    public void runIntake() {
        intakeRunning = true;
        if (!Configurables.doSimulation)
            intakeMotor.set(intakeSpeed);
        else intakeMotorSim.setPower(intakeSpeed);
    }

    public void stopIntake() {
        intakeRunning = false;
        if (!Configurables.doSimulation)
            intakeMotor.stopMotor();
        else intakeMotorSim.setPower(0);
    }

    public void invertIntake() {
        direction = !direction;
        intakeMotor.setInverted(direction);
    }

    public double getVelocityRPM() {
        if (!Configurables.doSimulation) {
            return intakeMotor.getCorrectedVelocity();
        } else {
            return intakeSpeed * 1100;
        }
    }

    public double getPower() {
        if (!Configurables.doSimulation) {
            return intakeMotor.getRawPower();
        } else {
            return intakeMotorSim.getPower();
        }
    }

    @Override
    public void periodic() {
        if (Configurables.doSimulation)
            intakeMotorSim.update(intakeMotorSim.getPower(), simDt);

    }

    public Command runIntakeCommand() {
        return Commands.runOnce(this::runIntake, this);
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(this::stopIntake);

    }

    public Command invertIntakeCommand() {
        return Commands.runOnce(this::invertIntake);
    }

    public Command armUpCommand() {
        return Commands.runOnce(() -> {
            leftArmTargetPosition = upPosition;
            rightArmTargetPosition = upPosition + rightTrimValue;
            leftArmServo.setPosition(leftArmTargetPosition);
            rightArmServo.setPosition(rightArmTargetPosition);
        });
    }

    public Command armDownCommand() {
        return Commands.runOnce(() -> {
            leftArmTargetPosition = downPosition;
            rightArmTargetPosition = downPosition + rightTrimValue;
            leftArmServo.setPosition(leftArmTargetPosition);
            rightArmServo.setPosition(rightArmTargetPosition);
        });
    }

    public Command armRejectNectarCommand() {
        return Commands.runOnce(() -> {
            leftArmTargetPosition = noNectarPosition;
            rightArmTargetPosition = noNectarPosition + rightTrimValue;
            leftArmServo.setPosition(leftArmTargetPosition);
            rightArmServo.setPosition(rightArmTargetPosition);
        });
    }

    public Command toggleArmPositionCommand() {
        return Commands.either(
                armUpCommand(),
                armUpCommand(),
                () -> leftArmTargetPosition == downPosition);
    }

    public Command stepArmDownCommand() {
        return Commands.runOnce((() -> stepArmPositionDown()));
    }

    public double stepArmPositionDown() {
        if (leftArmTargetPosition == upPosition) {
            leftArmTargetPosition = noNectarPosition;
        } else if (leftArmTargetPosition == noNectarPosition) {
            leftArmTargetPosition = downPosition;
        } else if (leftArmTargetPosition == downPosition) {
            leftArmTargetPosition = upPosition;
        }
        rightArmTargetPosition = leftArmTargetPosition + rightTrimValue;
        leftArmServo.setPosition(leftArmTargetPosition);
        rightArmServo.setPosition(rightArmTargetPosition);
        return leftArmTargetPosition;
    }

    public double getLeftArmPosition() {
        return leftArmTargetPosition;
    }

    public double getRightArmPosition() {
        return rightArmTargetPosition;
    }


    public void showTelemetry(TelemetryManager telemetryM) {
        telemetryM.addData("IntakePower", getPower());
        telemetryM.addData("IntakeRPM", getVelocityRPM());
        telemetryM.addData("IntakeRunning", intakeRunning);
        telemetryM.addData("LeftArmPosition", getLeftArmPosition());
        telemetryM.addData("RightArmPosition", getRightArmPosition());

    }


}