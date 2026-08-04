package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Full Robot TeleOp — Mecanum Drive (with heading/rotation) + Intake + Control + Outtake
 *
 * Drive Control:
 *   Right Joystick -> Forward/Back + Strafe
 *   Left  Joystick (X-axis) -> Rotate / change heading
 *
 * Mechanism Control:
 *   X -> Intake toggle (435 RPM on/off)
 *   Y -> Control motor cycle (Off -> 5 RPM -> 1000 RPM -> Off...)
 *   B -> Outtake toggle (5500 RPM on/off, reversed direction)
 *   D-Pad Up   -> Increase the currently active control stage's RPM by 20
 *   D-Pad Down -> Decrease the currently active control stage's RPM by 20
 */
@TeleOp(name = "Maybe" + "Robot TeleOp", group = "TeleOp")
public class ForWheelOpMode extends LinearOpMode {

    // ------------------- Drivetrain -------------------
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    private static final double MAX_SPEED = 1.0;
    private static final double DEADZONE  = 0.05;

    // Turning is usually more sensitive than translation — scale it down a bit.
    // Set to 1.0 if you want full-speed rotation.
    private static final double TURN_SENSITIVITY = 0.7;

    // ------------------- Mechanisms -------------------
    private DcMotorEx intake_motor;
    private DcMotorEx control_motor;
    private DcMotorEx outtake_motor;

    // Intake: goBILDA 5203 19.2:1, 435 RPM target
    private static final double INTAKE_TICKS_PER_REV = 384.5;
    private static final double INTAKE_TARGET_RPM = 435;
    private static final double INTAKE_TICKS_PER_SEC =
            (INTAKE_TARGET_RPM * INTAKE_TICKS_PER_REV) / 60.0;
    private boolean intakeOn = false;
    private boolean xPressedLast = false;

    // Control: goBILDA 5203 19.2:1, 3-stage cycle (0 -> 5 RPM -> 1000 RPM -> 0)
    private static final double CONTROL_TICKS_PER_REV = 384.5;
    private static final double CONTROL_RPM_STEP = 20; // amount to change per D-pad press

    // Both stage RPMs are adjustable at runtime via D-pad up/down.
    // D-pad affects whichever stage is currently active.
    private double controlRpmStage1 = 5;
    private double controlRpmStage2 = 1000;
    private int controlStage = 0; // 0 = off, 1 = stage-1 RPM, 2 = stage-2 RPM
    private boolean yPressedLast = false;
    private boolean dpadUpPressedLast = false;
    private boolean dpadDownPressedLast = false;

    // Outtake: goBILDA 5203 Yellow Jacket (no gearbox, 6000 RPM), target 5500 RPM
    private static final double OUTTAKE_TICKS_PER_REV = 28.0;
    private static final double OUTTAKE_TARGET_RPM = 10;
    private static final double OUTTAKE_TICKS_PER_SEC =
            (OUTTAKE_TARGET_RPM * OUTTAKE_TICKS_PER_REV) / 60.0;
    private boolean outtakeOn = false;
    private boolean bPressedLast = false;

    @Override
    public void runOpMode() {

        // ---------------- Hardware Map: Drivetrain ----------------
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftBackMotor   = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightBackMotor  = hardwareMap.get(DcMotor.class, "right_back_motor");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---------------- Hardware Map: Mechanisms ----------------
        intake_motor  = hardwareMap.get(DcMotorEx.class, "intake_motor");
        control_motor = hardwareMap.get(DcMotorEx.class, "control_motor");
        outtake_motor = hardwareMap.get(DcMotorEx.class, "outtake_motor");

        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        control_motor.setDirection(DcMotor.Direction.FORWARD);
        outtake_motor.setDirection(DcMotor.Direction.REVERSE);

        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        control_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        control_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        control_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized - waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ============= DRIVETRAIN (translation + rotation) =============
            double rawDrive  = -gamepad1.right_stick_y;
            double rawStrafe =  gamepad1.right_stick_x;
            double rawTurn   =  gamepad1.left_stick_x;

            double drive  = applyDeadzone(rawDrive);
            double strafe = applyDeadzone(rawStrafe);
            double turn   = applyDeadzone(rawTurn) * TURN_SENSITIVITY;

            double leftFrontPower  = drive + strafe + turn;
            double rightFrontPower = drive - strafe - turn;
            double leftBackPower   = drive - strafe + turn;
            double rightBackPower  = drive + strafe - turn;

            double maxPower = Math.max(
                    Math.max(Math.abs(leftFrontPower),  Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower),   Math.abs(rightBackPower))
            );
            if (maxPower > MAX_SPEED) {
                leftFrontPower  /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower   /= maxPower;
                rightBackPower  /= maxPower;
            }

            leftFrontPower  = Range.clip(leftFrontPower,  -MAX_SPEED, MAX_SPEED);
            rightFrontPower = Range.clip(rightFrontPower, -MAX_SPEED, MAX_SPEED);
            leftBackPower   = Range.clip(leftBackPower,   -MAX_SPEED, MAX_SPEED);
            rightBackPower  = Range.clip(rightBackPower,  -MAX_SPEED, MAX_SPEED);

            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);

            // ============= INTAKE (X toggle) =============
            boolean xPressed = gamepad1.x;
            if (xPressed && !xPressedLast) {
                intakeOn = !intakeOn;
            }
            xPressedLast = xPressed;
            intake_motor.setVelocity(intakeOn ? INTAKE_TICKS_PER_SEC : 0);

            // ============= CONTROL (Y cycle) =============
            boolean yPressed = gamepad1.y;
            if (yPressed && !yPressedLast) {
                controlStage = (controlStage + 1) % 3;
            }
            yPressedLast = yPressed;

            // ============= CONTROL RPM ADJUST (D-Pad Up/Down) =============
            // Adjusts whichever stage is currently active.
            boolean dpadUpPressed = gamepad1.dpad_up;
            boolean dpadDownPressed = gamepad1.dpad_down;
            double rpmDelta = 0;
            if (dpadUpPressed && !dpadUpPressedLast) {
                rpmDelta += CONTROL_RPM_STEP;
            }
            if (dpadDownPressed && !dpadDownPressedLast) {
                rpmDelta -= CONTROL_RPM_STEP;
            }
            dpadUpPressedLast = dpadUpPressed;
            dpadDownPressedLast = dpadDownPressed;

            if (rpmDelta != 0) {
                if (controlStage == 1) {
                    controlRpmStage1 = Math.max(0, controlRpmStage1 + rpmDelta);
                } else if (controlStage == 2) {
                    controlRpmStage2 = Math.max(0, controlRpmStage2 + rpmDelta);
                }
                // If controlStage == 0 (off), D-pad does nothing since there's no active RPM to adjust.
            }

            double controlTicksPerSecStage1 = (controlRpmStage1 * CONTROL_TICKS_PER_REV) / 60.0;
            double controlTicksPerSecStage2 = (controlRpmStage2 * CONTROL_TICKS_PER_REV) / 60.0;

            switch (controlStage) {
                case 1:
                    control_motor.setVelocity(controlTicksPerSecStage1);
                    break;
                case 2:
                    control_motor.setVelocity(controlTicksPerSecStage2);
                    break;
                default:
                    control_motor.setVelocity(0);
                    break;
            }

            // ============= OUTTAKE (B toggle) =============
            boolean bPressed = gamepad1.b;
            if (bPressed && !bPressedLast) {
                outtakeOn = !outtakeOn;
            }
            bPressedLast = bPressed;
            outtake_motor.setVelocity(outtakeOn ? OUTTAKE_TICKS_PER_SEC : 0);

            // ============= TELEMETRY =============
            telemetry.addData("-- Drivetrain -----------", "");
            telemetry.addData("Left  Front", "%.2f", leftFrontPower);
            telemetry.addData("Right Front", "%.2f", rightFrontPower);
            telemetry.addData("Left  Back",  "%.2f", leftBackPower);
            telemetry.addData("Right Back",  "%.2f", rightBackPower);
            telemetry.addData("Turn Input",  "%.2f", turn);
            telemetry.addData("-- Mechanisms -----------", "");
            telemetry.addData("Intake On", intakeOn);
            telemetry.addData("Control Stage", controlStage);
            telemetry.addData("Control Stage-1 RPM", "%.0f", controlRpmStage1);
            telemetry.addData("Control Stage-2 RPM", "%.0f", controlRpmStage2);
            telemetry.addData("Outtake On", outtakeOn);
            telemetry.update();
        }

        // Stop everything on exit
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        intake_motor.setVelocity(0);
        control_motor.setVelocity(0);
        outtake_motor.setVelocity(0);
    }

    /**
     * Applies a deadzone to a joystick axis and rescales the remaining range
     * back to [-1, 1] so there's no dead "jump" right past the deadzone edge.
     */
    private double applyDeadzone(double rawInput) {
        if (Math.abs(rawInput) < DEADZONE) {
            return 0.0;
        }
        return (rawInput - Math.signum(rawInput) * DEADZONE) / (1.0 - DEADZONE);
    }
}