package org.firstinspires.ftc.teamcode.subsystems; // Best practice: keep OpModes in an opmodes package

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Configurable
@TeleOp(name = "PIDF Tuner", group = "Tuning")
public class ShooterTuningOpMode extends LinearOpMode {

    // Dashboard Controls - Change these in FTC Dashboard while running
    public static boolean RUN_SHOOTER = true;
    public static double TEST_TARGET = 1000;
    Indexer indexer;
    Spindexer spindexer;

    // Quick preset buttons for common velocities
    public static boolean TEST_CLOSE = false;
    public static boolean TEST_MEDIUM = false;
    public static boolean TEST_FAR = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // IMPORTANT: Before running this, check your Driver Station!
        // Make sure you have a motor configured with the exact name "shooter"
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        indexer.Init(hardwareMap, telemetry, spindexer);

        telemetry.addLine("=================================");
        telemetry.addLine("  SHOOTER PIDF TUNING MODE");
        telemetry.addLine("=================================");
        telemetry.addLine("1. Open FTC Dashboard in browser");
        telemetry.addLine("2. Toggle RUN_SHOOTER to start");
        telemetry.addLine("3. Adjust TEST_TARGET velocity");
        telemetry.addLine("4. Tune kP, kV, kS live");
        telemetry.addLine("=================================");
        telemetry.addLine("Ready! Press PLAY to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Control shooter based on dashboard toggle
            if (gamepad1.dpad_down) {
                shooter.setTarget(TEST_TARGET);
                shooter.on(); // CRITICAL: We must explicitly turn it on
            } else {
                shooter.setTarget(0);
                shooter.off(); // CRITICAL: We must explicitly turn it off
            }
            if (gamepad1.dpad_up){
                indexer.Index();
            }

            // Apply velocity control math
            shooter.periodic();

            // Telemetry for dashboard graphs
            telemetry.addData("═══════════════════════════════", "");
            telemetry.addData("SHOOTER STATUS", RUN_SHOOTER ? "RUNNING ✓" : "STOPPED ✗");
            telemetry.addData("═══════════════════════════════", "");

            telemetry.addData("Target Velocity", shooter.getTarget());
            telemetry.addData("Actual Velocity", shooter.getVelocity());
            telemetry.addData("Error", shooter.getTarget() - shooter.getVelocity());

            telemetry.addData("═══════════════════════════════", "");
            telemetry.addData("PIDF Coefficients", "");

            // Fixed: Accessing static variables through the Class name, not the instance
            telemetry.addData("  kP", ShooterSubsystem.kP);
            telemetry.addData("  kV", ShooterSubsystem.kV);
            telemetry.addData("  kS", ShooterSubsystem.kS);

            telemetry.addData("═══════════════════════════════", "");
            telemetry.addData("Controls", "");
            telemetry.addData("  RUN_SHOOTER", RUN_SHOOTER ? "ON" : "OFF");
            telemetry.addData("  TEST_TARGET", TEST_TARGET);

            telemetry.update();
        }

        // Stop shooter safely when opmode ends
        shooter.off();
    }
}