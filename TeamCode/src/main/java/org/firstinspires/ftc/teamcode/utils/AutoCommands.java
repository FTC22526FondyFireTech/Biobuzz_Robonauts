package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.CatapultSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class AutoCommands {

    private final MecanumDriveSubsystem drive;
    private CatapultSubsystem catapultSubsystem;

    private ElapsedTime fireTimer = new ElapsedTime();

    public AutoCommands(MecanumDriveSubsystem drive, CatapultSubsystem catapultSubsystem) {
        this.drive = drive;
        this.catapultSubsystem = catapultSubsystem;
    }

    public Command shootCommand(){
        return catapultSubsystem.cycleCatapultCommand();
    }


}

