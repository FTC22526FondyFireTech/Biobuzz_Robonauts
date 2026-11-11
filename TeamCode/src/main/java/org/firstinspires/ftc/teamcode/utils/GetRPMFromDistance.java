package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.CatapultSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class GetRPMFromDistance {



    CatapultSubsystem nss;
    MecanumDriveSubsystem drive;

    RollingAverage pollenRa = new RollingAverage(10);
    RollingAverage nectarRa = new RollingAverage(10);






    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.hypot(XDiff, YDiff);
    }

}