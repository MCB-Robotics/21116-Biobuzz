
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "flywheelPIDF", group = "TeleOp")
public class flywheelPIDF extends OpMode {

    // =====================================
    // FLYWHEEL CONFIGURATION
    // =====================================

    private static final double TARGET_VELOCITY = 2700.0;
    private static final double VELOCITY_TOLERANCE = 100.0;

    // Encoder ticks per motor revolution
    // Assumes bare REV HD Hex motor
    private static final double TICKS_PER_REV = 28.0;

    // Starting PIDF values
    private double p = 0.0;
    private double i = 0.0;
    private double d = 0.0;
    private double f = 32767.0 / TARGET_VELOCITY;

    // PIDF adjustment sizes
    private static final double P_STEP = 0.1;
    private static final double I_STEP = 0.001;
    private static final double D_STEP = 0.01;
    private static final double F_STEP = 0.1;


    // =====================================
    // MOTOR VARIABLES
    // =====================================

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private double targetVelocity = 0.0;


    // =====================================
    // LIVE TUNING VARIABLES
    // =====================================

    // 0 = P, 1 = I, 2 = D, 3 = F
    private int selectedCoefficient = 0;

    private boolean previousLeft = false;
    private boolean previousRight = false;
    private boolean previousUp = false;
    private boolean previousDown = false;


    // =====================================
    // INITIALISE
    // =====================================

    @Override
    public void init() {

        motor1 = hardwareMap.get(
                DcMotorEx.class, "flywheel1");

        motor2 = hardwareMap.get(
                DcMotorEx.class, "flywheel2");

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setVelocity(0);
        motor2.setVelocity(0);

        applyPIDF();

        telemetry.addLine("BIOBUZZ Flywheel Initialised");
        telemetry.update();
    }


    // =====================================
    // APPLY PIDF VALUES
    // =====================================

    private void applyPIDF() {

        PIDFCoefficients coefficients =
                new PIDFCoefficients(p, i, d, f);

        motor1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                coefficients
        );

        motor2.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                coefficients
        );
    }


    // =====================================
    // SELECTED COEFFICIENT NAME
    // =====================================

    private String getSelectedName() {

        switch (selectedCoefficient) {

            case 0:
                return "P";

            case 1:
                return "I";

            case 2:
                return "D";

            case 3:
                return "F";

            default:
                return "Unknown";
        }
    }


    // =====================================
    // ADJUST SELECTED COEFFICIENT
    // =====================================

    private void adjustCoefficient(double direction) {

        switch (selectedCoefficient) {

            case 0:
                p = Math.max(0.0, p + direction * P_STEP);
                break;

            case 1:
                i = Math.max(0.0, i + direction * I_STEP);
                break;

            case 2:
                d = Math.max(0.0, d + direction * D_STEP);
                break;

            case 3:
                f = Math.max(0.0, f + direction * F_STEP);
                break;
        }

        // Immediately update both motor controllers
        applyPIDF();
    }


    // =====================================
    // LIVE PIDF TUNING CONTROLS
    // =====================================

    private void updateTuningControls() {

        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;

        // Select previous coefficient
        if (left && !previousLeft) {

            selectedCoefficient--;

            if (selectedCoefficient < 0) {
                selectedCoefficient = 3;
            }
        }

        // Select next coefficient
        if (right && !previousRight) {

            selectedCoefficient++;

            if (selectedCoefficient > 3) {
                selectedCoefficient = 0;
            }
        }

        // Increase selected coefficient
        if (up && !previousUp) {
            adjustCoefficient(1.0);
        }

        // Decrease selected coefficient
        if (down && !previousDown) {
            adjustCoefficient(-1.0);
        }

        previousLeft = left;
        previousRight = right;
        previousUp = up;
        previousDown = down;
    }


    // =====================================
    // MAIN LOOP
    // =====================================

    @Override
    public void loop() {

        // =====================================
        // GAMEPAD 1: FLYWHEEL CONTROL
        // =====================================

        // A: Start flywheels
        if (gamepad1.a) {

            targetVelocity = TARGET_VELOCITY;

            motor1.setVelocity(-targetVelocity);
            motor2.setVelocity(targetVelocity);
        }

        // B: Stop flywheels
        if (gamepad1.b) {

            targetVelocity = 0.0;

            motor1.setVelocity(0);
            motor2.setVelocity(0);
        }


        // =====================================
        // GAMEPAD 2: PIDF TUNING
        // =====================================

        updateTuningControls();


        // =====================================
        // GET MOTOR VELOCITIES
        // =====================================

        // Absolute values account for opposite motor directions
        double velocity1 =
                Math.abs(motor1.getVelocity());

        double velocity2 =
                Math.abs(motor2.getVelocity());

        double averageVelocity =
                (velocity1 + velocity2) / 2.0;


        // =====================================
        // CONVERT VELOCITY TO RPM
        // =====================================

        double motor1RPM =
                velocity1 * 60.0 / TICKS_PER_REV;

        double motor2RPM =
                velocity2 * 60.0 / TICKS_PER_REV;

        double averageRPM =
                (motor1RPM + motor2RPM) / 2.0;

        double targetRPM =
                targetVelocity * 60.0 / TICKS_PER_REV;


        // =====================================
        // CALCULATE ERROR
        // =====================================

        double error =
                targetVelocity - averageVelocity;

        double rpmError =
                targetRPM - averageRPM;


        // =====================================
        // CHECK TARGET SPEED
        // =====================================

        boolean atSpeed =
                targetVelocity > 0
                        && Math.abs(velocity1 - targetVelocity)
                        <= VELOCITY_TOLERANCE
                        && Math.abs(velocity2 - targetVelocity)
                        <= VELOCITY_TOLERANCE;


        // =====================================
        // TELEMETRY
        // =====================================

        telemetry.addLine("=== BIOBUZZ FLYWHEEL ===");

        telemetry.addLine("--- VELOCITY (TICKS/SEC) ---");

        telemetry.addData("Target Velocity", "%.1f",
                targetVelocity);

        telemetry.addData("Motor 1 Velocity", "%.1f",
                velocity1);

        telemetry.addData("Motor 2 Velocity", "%.1f",
                velocity2);

        telemetry.addData("Average Velocity", "%.1f",
                averageVelocity);

        telemetry.addData("Velocity Error", "%.1f",
                error);


        telemetry.addLine("--- FLYWHEEL RPM ---");

        telemetry.addData("Target RPM", "%.1f",
                targetRPM);

        telemetry.addData("Motor 1 RPM", "%.1f",
                motor1RPM);

        telemetry.addData("Motor 2 RPM", "%.1f",
                motor2RPM);

        telemetry.addData("Average RPM", "%.1f",
                averageRPM);

        telemetry.addData("RPM Error", "%.1f",
                rpmError);


        telemetry.addLine("--- PERFORMANCE ---");

        telemetry.addData("Motor Difference (RPM)", "%.1f",
                Math.abs(motor1RPM - motor2RPM));

        telemetry.addData("At Target Speed", atSpeed);


        telemetry.addLine("--- LIVE PIDF TUNING ---");

        telemetry.addData("Selected Coefficient",
                getSelectedName());

        telemetry.addData("P", "%.4f", p);
        telemetry.addData("I", "%.4f", i);
        telemetry.addData("D", "%.4f", d);
        telemetry.addData("F", "%.4f", f);


        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("GAMEPAD 1: A = Start | B = Stop");
        telemetry.addLine("GAMEPAD 2: LEFT/RIGHT = Select PIDF");
        telemetry.addLine("GAMEPAD 2: UP/DOWN = Adjust PIDF");

        telemetry.update();
    }


    // =====================================
    // STOP
    // =====================================

    @Override
    public void stop() {

        targetVelocity = 0.0;

        motor1.setVelocity(0);
        motor2.setVelocity(0);
    }
}
