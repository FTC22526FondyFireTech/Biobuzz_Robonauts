package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.TestSubsysytem;

import java.util.Locale;


//@Autonomous(name = "Blank")
@TeleOp(name = "Sim Cat Test")
//@Disabled

public class SimMotorTestOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    TestSubsysytem testSubsysytem;
    GamepadEx driverGamepad;
    ElapsedTime simTimer;
    double lastTime;

    double dt;


    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f2(double v) {
        return String.format(Locale.US, "%.2f", v);
    }

    static String f1(double v) {
        return String.format(Locale.US, "%.1f", v);
    }

    double plusLimitDegrees = 120;
    double minusLimitDegrees = -10;

    @Override
    public void initialize() {
        testSubsysytem = new TestSubsysytem();
        driverGamepad = new GamepadEx(gamepad1);
        simTimer = new ElapsedTime();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        testSubsysytem.leftMotorSim.resetEncoder();
        telemetryM.update(telemetry);


        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(testSubsysytem.positionCatapultCommand(90));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(testSubsysytem.positionCatapultCommand(0));

       // driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)



        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenHeld(testSubsysytem.jogSimCatapultCommand(.2))
                .whenReleased(testSubsysytem.jogSimCatapultCommand(0.));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenHeld(testSubsysytem.jogSimCatapultCommand(-.2))
                .whenReleased(testSubsysytem.jogSimCatapultCommand(0.));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(testSubsysytem.resetSimEncoderCommand());


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        simTimer.reset();
        while (!isStopRequested() && opModeIsActive()) {
            run();

            double currentTime = simTimer.seconds();
            dt = currentTime - lastTime;
            lastTime = currentTime;

            testSubsysytem.leftMotorSim.update(testSubsysytem.leftMotorSim.getPower(), dt);

//            if (testSubsysytem.leftMotorSim.getLoadDegrees() > plusLimitDegrees && testSubsysytem.leftMotorSim.getPower() > 0
//                    || testSubsysytem.leftMotorSim.getLoadDegrees() < minusLimitDegrees && testSubsysytem.leftMotorSim.getPower() < 0)
//                testSubsysytem.leftMotorSim.setPower(0);

            telemetryM.addData("TST", testSubsysytem.test);
            telemetryM.addData("PID P", f3(testSubsysytem.pidController.getP()));
            telemetryM.addData("Output", f2(testSubsysytem.output));
            telemetryM.addData("Setpoint", f1(testSubsysytem.pidController.getSetPoint()));
            telemetryM.addData("SimDeg", f1(testSubsysytem.leftMotorSim.getLoadDegrees()));
            telemetryM.addData("AtGoal", (testSubsysytem.atGoal()));
            telemetryM.addData("SetpointPos", f1(testSubsysytem.targetDegreesPerSec));
            telemetryM.addData("Poserr", f1(testSubsysytem.pidController.getPositionError()));
            telemetryM.addData("VelErr", f1(testSubsysytem.pidController.getVelocityError()));


            telemetryM.addData("SimRevs", f2(testSubsysytem.leftMotorSim.getMotorRevs()));
            telemetryM.addData("SimPower", f2(testSubsysytem.leftMotorSim.getPower()));
            telemetryM.addData("SimRPM", f1(testSubsysytem.leftMotorSim.getMotorRPM()));
            telemetryM.addData("SimAcc", f3(testSubsysytem.leftMotorSim.acceleration));
            telemetryM.addData("Sim Tgt Vel", f1(testSubsysytem.leftMotorSim.targetVelocityRPM));
            telemetryM.addData("SimTicks", testSubsysytem.leftMotorSim.getEncoderTicks());
            telemetryM.addData("SimTicksTF", f2(testSubsysytem.leftMotorSim.ticksThisFrame));
            telemetryM.addData("SimTicksPerSec", f3(testSubsysytem.leftMotorSim.ticksPerSecond));
            telemetryM.addData("DT", f3(dt));
            telemetryM.update(telemetry);
        }
        reset();
        testSubsysytem.leftMotorSim.setPower(0);
    }


}