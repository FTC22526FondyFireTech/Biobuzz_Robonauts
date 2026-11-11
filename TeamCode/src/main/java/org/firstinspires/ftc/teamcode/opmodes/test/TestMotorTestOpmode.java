package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.TestMotorSubsysytem;

import java.util.Locale;


//@Autonomous(name = "Blank")
@TeleOp(name = "Cat Motor Test")
//@Disabled

public class TestMotorTestOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    TestMotorSubsysytem testMotorSubsysytem;
    GamepadEx driverGamepad;

    private double currentPower;

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


    @Override
    public void initialize() {
        testMotorSubsysytem = new TestMotorSubsysytem(hardwareMap);


        driverGamepad = new GamepadEx(gamepad1);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.update(telemetry);

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(testMotorSubsysytem.positionCatapultCommand(90));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(testMotorSubsysytem.positionCatapultCommand(0));

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(testMotorSubsysytem.cycleCatapultCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(Commands.sequence(
                        Commands.runOnce(() -> testMotorSubsysytem.stateTimer.reset()),
                        Commands.runOnce(() -> testMotorSubsysytem.state = 0),
                        Commands.runOnce(() -> testMotorSubsysytem.runCatapult = true)));

        ;
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(Commands.runOnce(() -> testMotorSubsysytem.stopCatapult = true));


        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenHeld(testMotorSubsysytem.jogTestMotorCommand(.2))
                .whenReleased(testMotorSubsysytem.jogTestMotorCommand(0.));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenHeld(testMotorSubsysytem.jogTestMotorCommand(-.2))
                .whenReleased(testMotorSubsysytem.jogTestMotorCommand(0.));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(testMotorSubsysytem.resetEncoderCommand());


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        testMotorSubsysytem.power_on = true;

        while (!isStopRequested() && opModeIsActive()) {
            run();


            testMotorSubsysytem.showTelemetry(telemetryM);
//            telemetryM.addData("MAxOut", testMotorSubsysytem.pidController.getMaxOutput());
//            telemetryM.addData("CatDeg", f3(testMotorSubsysytem.getCatapultAngleDegrees()));
//            telemetryM.addData("Motor RPM", f3(testMotorSubsysytem.getCatapultVelocity()));
//            telemetryM.addData("Motor AMPS", f3(testMotorSubsysytem.getMotorAmps()));


            telemetryM.update(telemetry);
        }
        reset();

    }


}