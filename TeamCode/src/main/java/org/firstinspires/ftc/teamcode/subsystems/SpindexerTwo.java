package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class SpindexerTwo {
    public DcMotor spindexer;
    private String motorName = "motor2";

    // PID Coefficients - SIGNIFICANTLY INCREASED
    public static double Kp = 0.07;  // Increased 3x - stronger correction
    public static double Ki = 0.0001;    // DISABLED - was causing drift
    public static double Kd = 0.1;  // Increased 3x - better damping
    public static double power = 0.1;

    private double ticksPerRotation = 384.5;
    private double targetPositionMultiple = ticksPerRotation / 3;
    public double positionOne = targetPositionMultiple;
    public double positionTwo = targetPositionMultiple * 2;
    public double positionThree = targetPositionMultiple * 3;
    public static double sT = 20;

    // PID Variables
    private double lastError = 0;
    private double integralSum = 0;
    private final Timer pidTimer = new Timer();
    private final Timer integralTimer = new Timer();

    // PID tuning parameters
    private static final double POSITION_TOLERANCE = 3.0; // Slightly looser tolerance
    private static final double MAX_INTEGRAL = 50;        // Reduced max integral
    private static final double MAX_POWER = 0.8;          // Increased max power
    private static final double MIN_POWER = -0.8;         // Increased min power

    // Target position variable for PID
    private int targetPosTicks = 0;

    public int currentPosition = 1;
    public int targetPositionIndex = 1;
    public int[] posStates = {-1, -1, -1};

    // Track rotations to always go forward
    private int rotationCount = 0; // How many full rotations we've done

    // Debouncing for button presses
    private final Timer moveTimer = new Timer();
    private static final double MIN_MOVE_DELAY = 0.3; // 300ms between moves

    public void getMotor(String nameOfMotor) {
        motorName = nameOfMotor;
    }

    // Initializing the motor with fresh values (resets encoder)
    public void freshInit(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(DcMotor.class, motorName);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize PID
        targetPosTicks = 0;
        currentPosition = 1;
        targetPositionIndex = 1;
        rotationCount = 0;
        lastError = 0;
        integralSum = 0;
        pidTimer.resetTimer();
        moveTimer.resetTimer();
    }

    public void dataInit(HardwareMap hardwareMap){
        spindexer = hardwareMap.get(DcMotor.class, motorName);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize PID and determine current position
        int currentTicks = spindexer.getCurrentPosition();
        targetPosTicks = currentTicks;

        // Figure out rotation count and position from encoder value
        rotationCount = (int)(currentTicks / ticksPerRotation);
        double remainderTicks = currentTicks % ticksPerRotation;

        determineCurrentPosition(remainderTicks);
        targetPositionIndex = currentPosition;

        lastError = 0;
        integralSum = 0;
        pidTimer.resetTimer();
        moveTimer.resetTimer();
    }

    public void setTargetPosition(int ticks) {
        targetPosTicks = ticks;
        // Reset integral when target changes to prevent windup
        integralSum = 0;
    }

    // Main PID update loop - MUST be called repeatedly in your OpMode loop
    public void update(){
        double currentPos = spindexer.getCurrentPosition();
        double error = targetPosTicks - currentPos;

        // Get time delta
        double deltaTime = pidTimer.getElapsedTime();

        // Prevent division by zero on first run
        if (deltaTime == 0) {
            pidTimer.resetTimer();
            return;
        }

        // Calculate Derivative
        double derivative = (error - lastError) / deltaTime;

        // Calculate Integral with anti-windup (only if Ki is non-zero)
        if (Ki > 0 && Math.abs(integralSum) < MAX_INTEGRAL) {
            integralSum += error * deltaTime;
        }

        // Reset integral if we're very close to target
//        if (Math.abs(error) < POSITION_TOLERANCE) {
//            integralSum = 0;
//        }

        // Calculate PID Power
        if (Math.abs(targetPosTicks - spindexer.getCurrentPosition()) > POSITION_TOLERANCE){
            power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        }
        else {
            power = (0.01 * error) + (0 * integralSum) + (0.05 * derivative);
        }




        // Clamp power to safe limits
        power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));

        // Apply power to motor
        spindexer.setPower(power);

        // Update currentPosition when we reach target
        if (isAtTarget()){
            currentPosition = targetPositionIndex;
        }
        if (Math.abs(targetPosTicks - spindexer.getCurrentPosition()) < 3 && integralTimer.getElapsedTime() > 500){
            integralSum = 0;
            integralTimer.resetTimer();
        }

        // Update for next iteration
        lastError = error;
        pidTimer.resetTimer();
    }

    // Determine which position we're at based on ticks within a rotation
    private void determineCurrentPosition(double ticksInRotation) {
        double dist1 = Math.abs(ticksInRotation - positionOne);
        double dist2 = Math.abs(ticksInRotation - positionTwo);
        double dist3 = Math.abs(ticksInRotation - positionThree);

        if (dist1 < dist2 && dist1 < dist3) {
            currentPosition = 1;
        } else if (dist2 < dist3) {
            currentPosition = 2;
        } else {
            currentPosition = 3;
        }
    }

    // Check if we're at the target position within tolerance
    public boolean isAtTarget() {
        return Math.abs(targetPosTicks - spindexer.getCurrentPosition()) < POSITION_TOLERANCE;
    }

    public boolean isAtPos(int pos) {
        return currentPosition == pos && isAtTarget();
    }

    public void GoToPos(int newPos, boolean priority) {
        // Prevent rapid position changes unless priority
        if (!priority) {
            if (targetPositionIndex == newPos) {
                return; // Already going there
            }
            if (moveTimer.getElapsedTime() < MIN_MOVE_DELAY) {
                return; // Too soon since last move
            }
        }

        // Calculate target ticks - ALWAYS MOVE FORWARD (counter-clockwise)
        int targetTicks = 0;

        // If going to a lower position number, we need to add a full rotation
        if (newPos < targetPositionIndex) {
            rotationCount++; // Moving to next rotation cycle
        }

        // Calculate absolute tick position
        if (newPos == 1) {
            targetTicks = (int)(rotationCount * ticksPerRotation + positionOne);
        } else if (newPos == 2) {
            targetTicks = (int)(rotationCount * ticksPerRotation + positionTwo);
        } else if (newPos == 3) {
            targetTicks = (int)(rotationCount * ticksPerRotation + positionThree);
        }

        // Set the target for the PID loop to follow
        setTargetPosition(targetTicks);
        targetPositionIndex = newPos;
        moveTimer.resetTimer();
    }

    public void GoToPose(int newPos, boolean torf) {
        GoToPos(newPos, torf);
    }

    // Only need counter-clockwise rotation now!
    public void rotateCounterclockwise() {
        int newPos = (targetPositionIndex % 3) + 1; // 1->2, 2->3, 3->1
        GoToPos(newPos, false);
    }

    // Keep this for compatibility but just call counter-clockwise
    public void rotateClockwise() {
        rotateCounterclockwise(); // Always rotate counter-clockwise
    }

    public void updateCurrentSlotState(String state) {
        if (state.equals("GREEN")) {
            posStates[currentPosition - 1] = 1;
        } else if (state.equals("PURPLE")) {
            posStates[currentPosition - 1] = 2;
        } else {
            posStates[currentPosition - 1] = -1;
        }
    }

    public boolean isSpindexerFull() {
        for (int i = 0; i < posStates.length; i++) {
            if (posStates[i] <= 0) {
                return false;
            }
        }
        return true;
    }

    public void rotateEmptySlot() {
        for (int i = 0; i < posStates.length; i++) {
            if (posStates[i] <= 0) {
                GoToPos(i+1, true); // Use priority to override debounce
                return;
            }
        }
    }

    // Getter methods for debugging/tuning
    public int getTargetPosition() {
        return targetPosTicks;
    }

    public int getCurrentPosition() {
        return spindexer.getCurrentPosition();
    }

    public int getCurrentPositionIndex() {
        return currentPosition;
    }

    public int getTargetPositionIndex() {
        return targetPositionIndex;
    }

    public double getCurrentError() {
        return targetPosTicks - spindexer.getCurrentPosition();
    }

    public double getIntegralSum() {
        return integralSum;
    }

    public double getLastPower() {
        return power;
//        return (Kp * getCurrentError()) + (Ki * integralSum) + (Kd * (getCurrentError() - lastError));
    }

    public int getRotationCount() {
        return rotationCount;
    }
}