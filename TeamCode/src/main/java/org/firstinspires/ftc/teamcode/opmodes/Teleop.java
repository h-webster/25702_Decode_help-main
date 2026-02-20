package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Ensure this points to your actual Constants file
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@TeleOp(name = "Teleop", group = "Drive")
public class Teleop extends OpMode {

    private Follower follower;
    private DcMotor shooter, spinner;

    private final Spindexer spindexer = new Spindexer();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();

    private double currentShooterPower = 0.0;

    @Override
    public void init() {
        // Initialize the Follower (This sets up your drivetrain motors automatically)
        follower = Constants.createFollower(hardwareMap);

        // Initialize other hardware
        spinner = hardwareMap.get(DcMotor.class, "motor2");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Subsystem Inits
        intake.Init(telemetry, hardwareMap);
        spindexer.init(hardwareMap, true);
        indexer.Init(hardwareMap, telemetry, spindexer);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // Pedro Pathing built-in TeleOp drive
        driveMecanum();

        // ==== Intake ====
        if (gamepad1.left_trigger > 0.05) {
            intake.spinOut();
        } else if (gamepad1.right_trigger > 0.05) {
            intake.stop();
        } else {
            intake.spinIn();
        }

        // ==== Shooter Controls ====
        // Note: Make sure your gamepad buttons (squareWasPressed) are supported
        // or use the standard boolean checks:
        if (gamepad1.square) {
            setShooter(-0.69);
        } else if (gamepad1.triangle) {
            setShooter(-0.63);
        } else if (gamepad1.cross) {
            setShooter(-0.77);
        }

        // ==== Indexer & Spindexer ====
        if (gamepad1.circle) {
            indexer.Index();
        }

        indexer.Update();
        spindexer.update();

        // Manual Spindexer logic
        if (indexer.currentState == Indexer.State.IDLE) {
            if (gamepad1.left_bumper) {
                spindexer.rotateCounterclockwise();
            }
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    private void setShooter(double power) {
        if (power == currentShooterPower) {
            currentShooterPower = 0;
        } else {
            currentShooterPower = power;
        }
        shooter.setPower(currentShooterPower);
    }

    private void driveMecanum() {
        // Pedro's setTeleOpDrive uses (forward, strafe, turn, useRobotCentric)
        follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
    }
}