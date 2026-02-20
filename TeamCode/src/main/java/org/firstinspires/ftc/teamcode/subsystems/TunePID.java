package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tune Spindexer PID")
public class TunePID extends LinearOpMode {

    private SpindexerTwo spindexer;

    @Override
    public void runOpMode() {
        // Initialize the spindexer
        spindexer = new SpindexerTwo();
        spindexer.freshInit(hardwareMap);
        spindexer.dataInit(hardwareMap);

        telemetry.addLine("Ready to tune!");
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD_UP: Position 1");
        telemetry.addLine("  DPAD_LEFT: Position 2");
        telemetry.addLine("  DPAD_RIGHT: Position 3");
        telemetry.addLine("  A: Rotate Clockwise");
        telemetry.addLine("  B: Rotate Counter-Clockwise");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // CRITICAL: Call update() every loop!
            spindexer.update();

            // Controls
            if (gamepad1.dpad_up) {
                spindexer.GoToPose(1, true);
            } else if (gamepad1.dpad_left) {
                spindexer.GoToPose(2, true);
            } else if (gamepad1.dpad_right) {
                spindexer.GoToPose(3, true);
            }

            if (gamepad1.a) {
                spindexer.rotateClockwise();
            } else if (gamepad1.b) {
                spindexer.rotateCounterclockwise();
            }

            // Telemetry for tuning
            telemetry.addData("Current Pos", spindexer.getCurrentPosition());
            telemetry.addData("Target Pos", spindexer.getTargetPosition());
            telemetry.addData("Error", spindexer.getCurrentError());
            telemetry.addData("At Target?", spindexer.isAtTarget());
            telemetry.addData("Position Index", spindexer.currentPosition);
            telemetry.addData("Last known power", spindexer.getLastPower());
            telemetry.addLine();
            telemetry.addLine("=========================================");
            telemetry.addLine();
            telemetry.addData("Kp", Spindexer.Kp);
            telemetry.addData("Ki", Spindexer.Ki);
            telemetry.addData("Kd", Spindexer.Kd);
            telemetry.addData("Integral Sum", spindexer.getIntegralSum());
            telemetry.update();
        }
    }
}