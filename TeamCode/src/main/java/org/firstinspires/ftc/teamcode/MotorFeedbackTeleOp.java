package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

@TeleOp(name = "BIOBUZZ Motor Lab PS4", group = "Testing")
public class MotorFeedbackTeleOp extends LinearOpMode {

    // =====================================================================
    // SETTINGS - edit these for your setup
    // =====================================================================

    // Motor order: the order D-pad LEFT/RIGHT cycles through.
    // Use the names EXACTLY as typed in the Driver Station robot configuration.
    // Leave empty {} to list all motors alphabetically.
    static final String[] MOTOR_ORDER = {
            // "frontLeft", "frontRight", "backLeft", "backRight",
    };

    // true  = only test the motors named in MOTOR_ORDER
    // false = also add any other configured motors after them (alphabetical)
    static final boolean ONLY_LISTED_MOTORS = false;

    // Drivetrain
    static final double GEAR_RATIO        = 1.0;
    static final double WHEEL_DIAMETER_MM = 100.0;

    // Motor data - CHECK AGAINST YOUR MOTOR'S DATASHEET
    static final double TICKS_PER_REV   = 28.0;
    static final double FREE_SPEED_RPM  = 6000.0;
    static final double STALL_TORQUE_NM = 0.105;
    static final double NOMINAL_VOLTAGE = 12.0;

    // Smoothing
    static final double FILTER_TIME_S = 0.3;

    // Power level scale and step sizes
    static final double MIN_LEVEL   = 0.0;
    static final double MAX_LEVEL   = 10.0;
    static final double FINE_STEP   = 0.05;
    static final double MEDIUM_STEP = 0.10;
    static final double COARSE_STEP = 1.00;

    // =====================================================================

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        // Bulk reads make encoder velocity reads much faster.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Battery voltage sensor.
        VoltageSensor battery = null;
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            battery = hardwareMap.voltageSensor.iterator().next();
        }

        // Find every configured motor, keyed by its config name.
        Map<String, DcMotorEx> found = new TreeMap<>();

        for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) {
            found.put(
                    hardwareMap.getNamesOf(motor).iterator().next(),
                    motor
            );
        }

        // Build the cycling order.
        List<DcMotorEx> motors = new ArrayList<>();
        List<String> names = new ArrayList<>();
        List<String> missing = new ArrayList<>();

        for (String n : MOTOR_ORDER) {
            DcMotorEx m = found.remove(n);

            if (m != null) {
                motors.add(m);
                names.add(n);
            } else if (!names.contains(n)) {
                missing.add(n);
            }
        }

        if (!ONLY_LISTED_MOTORS) {
            for (Map.Entry<String, DcMotorEx> e : found.entrySet()) {
                motors.add(e.getValue());
                names.add(e.getKey());
            }
        }

        // Initialise motors.
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (motors.isEmpty()) {
            telemetry.addLine("No DcMotorEx motors found.");
            telemetry.addLine("Check the REV hardware config and MOTOR_ORDER.");
            telemetry.update();

            waitForStart();
            return;
        }

        int count = motors.size();

        double[] level      = new double[count];
        double[] avgRPM     = new double[count];
        double[] avgCurrent = new double[count];

        boolean[] enabled   = new boolean[count];

        // NEW: Direction tracking for each motor.
        // false = forward
        // true  = reverse
        boolean[] reversed  = new boolean[count];

        boolean[] rpmPrimed = new boolean[count];
        boolean[] curPrimed = new boolean[count];

        double avgVoltage = NOMINAL_VOLTAGE;
        boolean voltPrimed = false;

        int selected = 0;
        int lastSelected = -1;

        boolean killLatched = false;

        boolean prevCross = false;
        boolean prevCircle = false;
        boolean prevTriangle = false;

        // NEW: Track previous Square button state.
        boolean prevSquare = false;

        boolean prevOptions = false;

        boolean prevUp = false;
        boolean prevDown = false;
        boolean prevLeft = false;
        boolean prevRight = false;

        // =================================================================
        // TELEMETRY INSTRUCTIONS
        // =================================================================

        telemetry.addLine("BIOBUZZ MOTOR LAB - PS4");
        telemetry.addData("Motors detected", count);

        for (int i = 0; i < count; i++) {
            telemetry.addLine("  " + (i + 1) + ". " + names.get(i));
        }

        if (!missing.isEmpty()) {
            telemetry.addData("WARNING not found", missing.toString());
        }

        telemetry.addLine("D-pad LEFT/RIGHT: Select motor");
        telemetry.addLine("D-pad UP/DOWN: Adjust power level (0-10)");
        telemetry.addLine("  hold L1: step 0.05");
        telemetry.addLine("  hold R1: step 0.1");
        telemetry.addLine("  neither: step 1");
        telemetry.addLine("OPTIONS: Skip to max (10)");
        telemetry.addLine("CROSS: Enable/disable selected motor");

        // NEW: Reverse direction control.
        telemetry.addLine(
                "SQUARE: Toggle direction (stops motor; re-enable after)"
        );

        telemetry.addLine("CIRCLE: GLOBAL KILL SWITCH");
        telemetry.addLine("TRIANGLE: Re-arm, motors stay off");

        telemetry.update();

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {

            // Time since last loop, used so smoothing doesn't depend on loop speed.
            double dt = loopTimer.seconds();
            loopTimer.reset();

            double alpha = 1.0 - Math.exp(-dt / FILTER_TIME_S);

            // =============================================================
            // CONTROLS
            // =============================================================

            // Global kill switch.
            boolean circle = gamepad1.circle;

            if (circle && !prevCircle) {

                killLatched = true;

                for (int i = 0; i < count; i++) {
                    enabled[i] = false;
                    level[i] = 0.0;
                    motors.get(i).setPower(0.0);
                }
            }

            prevCircle = circle;

            // Re-arm (motors stay off until re-enabled).
            boolean triangle = gamepad1.triangle;

            if (triangle && !prevTriangle) {
                killLatched = false;
            }

            prevTriangle = triangle;

            // Select motor.
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            if (left && !prevLeft) {
                selected = (selected - 1 + count) % count;
            }

            if (right && !prevRight) {
                selected = (selected + 1) % count;
            }

            prevLeft = left;
            prevRight = right;

            // When selection changes, restart the current average for the new motor.
            if (selected != lastSelected) {
                curPrimed[selected] = false;
                lastSelected = selected;
            }

            // Step size.
            double step;

            if (gamepad1.left_bumper) {
                step = FINE_STEP;
            } else if (gamepad1.right_bumper) {
                step = MEDIUM_STEP;
            } else {
                step = COARSE_STEP;
            }

            // Enable / disable.
            boolean cross = gamepad1.cross;

            if (cross && !prevCross && !killLatched) {
                enabled[selected] = !enabled[selected];
            }

            prevCross = cross;

            // =============================================================
            // NEW: FORWARD / REVERSE DIRECTION CONTROL
            // =============================================================

            boolean square = gamepad1.square;

            if (square && !prevSquare && !killLatched) {

                // Stop the currently selected motor first.
                enabled[selected] = false;

                // Reset its power level.
                level[selected] = 0.0;

                // Toggle its direction.
                reversed[selected] = !reversed[selected];
            }

            prevSquare = square;

            // =============================================================
            // Skip to max.
            // =============================================================

            boolean options = gamepad1.options;

            if (options && !prevOptions && !killLatched) {
                level[selected] = MAX_LEVEL;
                enabled[selected] = true;
            }

            prevOptions = options;

            // Adjust power level.
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (up && !prevUp && !killLatched) {
                level[selected] = Range.clip(
                        level[selected] + step,
                        MIN_LEVEL,
                        MAX_LEVEL
                );
            }

            if (down && !prevDown && !killLatched) {
                level[selected] = Range.clip(
                        level[selected] - step,
                        MIN_LEVEL,
                        MAX_LEVEL
                );
            }

            prevUp = up;
            prevDown = down;

            // =============================================================
            // BATTERY VOLTAGE
            // =============================================================

            if (battery != null) {

                double v = battery.getVoltage();

                if (v > 0) {

                    avgVoltage = voltPrimed
                            ? avgVoltage + alpha * (v - avgVoltage)
                            : v;

                    voltPrimed = true;
                }
            }

            // =============================================================
            // MOTORS
            // =============================================================

            for (int i = 0; i < count; i++) {

                DcMotorEx motor = motors.get(i);

                // Calculate power magnitude.
                double magnitude =
                        (!killLatched && enabled[i])
                                ? level[i] / MAX_LEVEL
                                : 0.0;

                // NEW: Apply direction.
                //
                // Forward = positive power.
                // Reverse = negative power.
                //
                // This is the main change that allows the motor
                // to physically rotate in the opposite direction.

                double command = reversed[i]
                        ? -magnitude
                        : magnitude;

                motor.setPower(command);

                // RPM from encoder (ticks per second -> revolutions per minute).
                double rawRPM =
                        Math.abs(motor.getVelocity()) * 60.0 / TICKS_PER_REV;

                avgRPM[i] = rpmPrimed[i]
                        ? avgRPM[i] + alpha * (rawRPM - avgRPM[i])
                        : rawRPM;

                rpmPrimed[i] = true;

                // =========================================================
                // OTHER MOTORS - SHORT TELEMETRY
                // =========================================================

                if (i != selected) {

                    telemetry.addData(
                            names.get(i),
                            "lvl %.2f | %s | %.0f RPM out",
                            level[i],
                            reversed[i] ? "REV" : "FWD",
                            avgRPM[i] / GEAR_RATIO
                    );

                    continue;
                }

                // =========================================================
                // SELECTED MOTOR - DETAILED READOUT
                // =========================================================

                // Current is not part of the bulk read,
                // so only read it for the selected motor.
                boolean currentAvailable = true;

                try {

                    double amps = motor.getCurrent(CurrentUnit.AMPS);

                    avgCurrent[i] = curPrimed[i]
                            ? avgCurrent[i] + alpha * (amps - avgCurrent[i])
                            : amps;

                    curPrimed[i] = true;

                } catch (Exception e) {

                    currentAvailable = false;
                }

                double motorRPM = avgRPM[i];

                double outputRPM = motorRPM / GEAR_RATIO;

                double wheelSpeed =
                        outputRPM * Math.PI
                                * (WHEEL_DIAMETER_MM / 1000.0)
                                / 60.0;

                // =========================================================
                // POWER AND TORQUE ESTIMATION
                // =========================================================

                // IMPORTANT:
                // Use absolute command here so the estimated power
                // remains positive in both forward and reverse.

                double effDuty =
                        Math.abs(command) * avgVoltage / NOMINAL_VOLTAGE;

                double motorTorque =
                        STALL_TORQUE_NM
                                * (effDuty - motorRPM / FREE_SPEED_RPM);

                motorTorque = Math.max(0.0, motorTorque);

                double omega = motorRPM * 2.0 * Math.PI / 60.0;

                double powerOut = motorTorque * omega;

                double outputTorque = motorTorque * GEAR_RATIO;

                // =========================================================
                // TELEMETRY DISPLAY
                // =========================================================

                telemetry.addLine("=== SELECTED MOTOR ===");

                telemetry.addData(
                        "Name",
                        "%s  (%d of %d)",
                        names.get(i),
                        i + 1,
                        count
                );

                telemetry.addData(
                        "State",
                        killLatched
                                ? "KILL LATCHED"
                                : enabled[i] ? "ENABLED" : "DISABLED"
                );

                // NEW: Show selected motor direction.
                telemetry.addData(
                        "Direction",
                        reversed[i] ? "REVERSE" : "FORWARD"
                );

                telemetry.addData(
                        "Power level",
                        "%.2f / 10",
                        level[i]
                );

                telemetry.addData(
                        "Step size",
                        "%.2f",
                        step
                );

                telemetry.addData(
                        "Motor command",
                        "%.3f",
                        command
                );

                telemetry.addLine();

                telemetry.addData(
                        "Motor RPM (avg)",
                        "%.0f",
                        motorRPM
                );

                telemetry.addData(
                        "Output RPM (avg)",
                        "%.1f  (%.2f:1)",
                        outputRPM,
                        GEAR_RATIO
                );

                telemetry.addData(
                        "Wheel speed",
                        "%.2f m/s  (%.0f mm wheel)",
                        wheelSpeed,
                        WHEEL_DIAMETER_MM
                );

                telemetry.addLine();

                telemetry.addData(
                        "Power output (est.)",
                        "%.1f W",
                        powerOut
                );

                telemetry.addData(
                        "Output torque (est.)",
                        "%.3f N·m",
                        outputTorque
                );

                if (currentAvailable) {

                    telemetry.addData(
                            "Current (avg)",
                            "%.2f A",
                            avgCurrent[i]
                    );

                } else {

                    telemetry.addLine("Current: unavailable");
                }

                telemetry.addData(
                        "Battery (avg)",
                        battery != null
                                ? String.format("%.2f V", avgVoltage)
                                : "no sensor"
                );

                telemetry.addLine("=====================");
            }

            telemetry.addData(
                    "Loop time",
                    "%.1f ms",
                    dt * 1000.0
            );

            telemetry.update();
        }

        // =============================================================
        // STOP EVERYTHING WHEN THE OPMODE ENDS
        // =============================================================

        for (DcMotorEx motor : motors) {
            motor.setPower(0.0);
        }
    }
}