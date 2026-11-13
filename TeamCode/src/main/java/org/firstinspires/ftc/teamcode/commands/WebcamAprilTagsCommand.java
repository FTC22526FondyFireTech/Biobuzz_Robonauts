package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WebcamAprilTagSubsysytem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

public class WebcamAprilTagsCommand extends CommandBase {


    TelemetryManager telemetryM;

    WebcamAprilTagSubsysytem wats;
    private int test;

    /**
     * The variable to store our instance of the vision portal.
     */

    public WebcamAprilTagsCommand(WebcamAprilTagSubsysytem wats) {
        this.wats = wats;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }


    @Override
    public void execute() {
        test++;
        telemetryM.addData("test", test);

        List<AprilTagDetection> currentDetections = wats.aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            wats.numberDetections = currentDetections.size();

            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    wats.singleMetaName = singleDet.metadata.name;
                    wats.singleMetaId = singleDet.metadata.id;
                    wats.currentTagZone = wats.zoneNames.indexOf(wats);
                    wats.aprilTagPoseFtc = detection.ftcPose;
                } else {
                    telemetryM.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                    telemetryM.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                wats.clusterMetaName = clusterDet.metadata.name;
                wats.pctClusterSeen = clusterDet.percentClusterFound;
                wats.aprilTagPoseFtc = detection.ftcPose;
            }
        }

    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
