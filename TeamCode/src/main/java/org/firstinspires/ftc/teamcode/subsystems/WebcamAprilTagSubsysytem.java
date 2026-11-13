package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.GlobalData;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class WebcamAprilTagSubsysytem extends SubsystemBase {
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    public ArrayList<String> zoneNames;
    public int numberDetections;
    public String singleMetaName = "";
    public int singleMetaId;
    public int currentTagZone;
    public int pctClusterSeen;
    public AprilTagPoseFtc aprilTagPoseFtc = new AprilTagPoseFtc(0, 0, 0, 0, 0, 0, 0, 0, 0);
    public String clusterMetaName = "";

    public Timing.Timer fallingTimer;


    public Timing.Timer raisedTimer;

    public int getExpectedZone() {
        return expectedZone;
    }

    public void setExpectedZone(int expectedZone) {
        this.expectedZone = expectedZone;
    }

    private int expectedZone;

    public boolean getZoneTagsSeen() {
        return zoneTagsSeen;
    }

    public void setZoneTagsSeen(boolean zoneTagsSeen) {
        this.zoneTagsSeen = zoneTagsSeen;
    }

    private boolean zoneTagsSeen;

    public boolean isHubFalling() {
        return hubFalling;
    }

    public void setHubFalling(boolean hubFalling) {
        this.hubFalling = hubFalling;
    }

    private boolean hubFalling;

    public boolean isHubRisen() {
        return hubRisen;
    }

    public void setHubRisen(boolean hubRisen) {
        this.hubRisen = hubRisen;
    }

    private boolean hubRisen;


    public WebcamAprilTagSubsysytem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        zoneNames = new ArrayList<String>();
        zoneNames.add("RED SCORING");
        zoneNames.add("RED AUDIENCE");
        zoneNames.add("BLUE AUDIENCE");
        zoneNames.add("BLUE SCORING");

        initAprilTag();

        fallingTimer = new Timing.Timer(5L, TimeUnit.SECONDS);
        raisedTimer = new Timing.Timer(20L, TimeUnit.SECONDS);
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam1"), aprilTag);

    }   // end

    public void setHiHiveStart() {
        if (GlobalData.isRedAlliance())
            setExpectedZone(1);
        else setExpectedZone(3);
    }

    public void setLoHiveStart() {
        if (GlobalData.isRedAlliance())
            setExpectedZone(0);
        else setExpectedZone(2);
    }

    @Override
    public void periodic() {

    }
}


