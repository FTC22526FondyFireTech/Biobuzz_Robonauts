package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.commands.WebcamAprilTagsCommand;
import org.firstinspires.ftc.teamcode.subsystems.WebcamAprilTagSubsysytem;

import java.util.Locale;

//@Autonomous(name = "Webcam tags")
@TeleOp(name = "WebCamTags")
@Disabled

public class WebcamTagsCommandOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    PanelsCameraStream pcs;

    WebcamAprilTagSubsysytem wats;

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
        wats = new WebcamAprilTagSubsysytem(hardwareMap);
        pcs = PanelsCameraStream.INSTANCE;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        pcs.startStream(wats.visionPortal, 200);
        new WebcamAprilTagsCommand(wats).schedule();
        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetryM.addData("# AprilTags Detected", wats.numberDetections);
            telemetryM.addData("Zone Name", wats.singleMetaName);
            telemetryM.addData("Zone ID", wats.singleMetaId);
            telemetryM.addData("Zone Number", wats.currentTagZone);
            telemetryM.addData("Cluster Name", wats.clusterMetaName);
            telemetryM.addData("Cluster Pct Seen", wats.pctClusterSeen);
            telemetryM.addData("X", f1(wats.aprilTagPoseFtc.x));
            telemetryM.addData("Y", f1(wats.aprilTagPoseFtc.y));
            telemetryM.addData("Z", f1(wats.aprilTagPoseFtc.z));
            telemetryM.addData("Pitch", f1(wats.aprilTagPoseFtc.pitch));
            telemetryM.addData("Roll", f1(wats.aprilTagPoseFtc.roll));
            telemetryM.addData("Yaw", f1(wats.aprilTagPoseFtc.yaw));
            telemetryM.update();
        }
        reset();
        wats.visionPortal.close();
        pcs.stopStream();
    }

}