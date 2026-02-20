package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class Robot {
    public final Intake intake;
    public final Indexer indexer;
    public final Spindexer spindexer;
    public final ShooterSubsystem shooter;
    public Alliance a;
    public Spinner spinner;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance, Spinner spinner){
        this.a = alliance;
        this.spinner = spinner;

        intake = new Intake();
        indexer = new Indexer();
        spindexer = new Spindexer();
        shooter = new ShooterSubsystem(hardwareMap);
    }
}
