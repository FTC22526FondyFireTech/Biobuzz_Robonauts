package org.firstinspires.ftc.teamcode.simulator.commands;

import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.CatapultSubsystem;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

public class AutoCommandsSim {

    private final MecanumDriveSubsystemSimulation driveSim;

    private final CatapultSubsystem catSim;

    public AutoCommandsSim(MecanumDriveSubsystemSimulation driveSim, CatapultSubsystem catSim) {
        this.driveSim = driveSim;
        this.catSim = catSim;
    }



    public Command autoFireAndReturnCommand() {
        return Commands.none();
    }


    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.sqrt(Math.pow(XDiff, 2) - Math.pow(YDiff, 2));
    }

    public double getDistanceFromOdometry() {
        return getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());
    }


}

