package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BioBuzz TeleOp", group = "Main")
public class BioBuzzTeleOp extends LinearOpMode {

    // ---------- Settings you can tweak ----------
    static final double OUTTAKE_RPM = 4500;
    static final double OUTTAKE_TICKS_PER_REV = 28;   // goBILDA 5203 6000rpm motor (bare). Change if geared.
    static final double OUTTAKE_TICKS_PER_SEC = OUTTAKE_RPM * OUTTAKE_TICKS_PER_REV / 60.0;

    static final double INTAKE_POWER = 1.0;
    static final double SIDE_SERVO_POWER = 1.0;
    static final double WINDMILL_POWER = 1.0;

    @Override
    public void runOpMode() {
        // ---------- Hardware (names must match your Robot Configuration) ----------
        DcMotor leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");     // Control Hub motor port 0
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");    // Control Hub motor port 1
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");  // Control Hub motor port 2
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake"); // Control Hub motor port 3

        CRServo rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo"); // Expansion Hub servo port 0
        CRServo windmillServo    = hardwareMap.get(CRServo.class, "windmillServo");    // Expansion Hub servo port 1
        CRServo leftIntakeServo  = hardwareMap.get(CRServo.class, "leftIntakeServo");  // Expansion Hub servo port 2

        // ---------- Directions ----------
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);   // flip if robot drives backwards
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Side servos spin opposite directions to pull balls toward the center
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // ---------- Motor modes ----------
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---------- Toggle state ----------
        boolean intakeOn = false, windmillOn = false, outtakeOn = false;
        boolean lastA = false, lastX = false, lastY = false;

        telemetry.addLine("Ready. A = Intake, X = Windmill, Y = Outtake");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // ---------- Drivetrain (arcade): left stick = forward/back, right stick = turn ----------
            double drive = -gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            double max = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
            leftDrive.setPower(leftPower / max);
            rightDrive.setPower(rightPower / max);

            // ---------- Toggles (rising edge detection) ----------
            if (gamepad1.a && !lastA) intakeOn = !intakeOn;     // Intake motor + side servos
            if (gamepad1.x && !lastX) windmillOn = !windmillOn; // Transfer windmill
            if (gamepad1.y && !lastY) outtakeOn = !outtakeOn;   // Outtake flywheel
            lastA = gamepad1.a;
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // ---------- Intake ----------
            intakeMotor.setPower(intakeOn ? INTAKE_POWER : 0);
            rightIntakeServo.setPower(intakeOn ? SIDE_SERVO_POWER : 0);
            leftIntakeServo.setPower(intakeOn ? SIDE_SERVO_POWER : 0);

            // ---------- Transfer windmill ----------
            windmillServo.setPower(windmillOn ? WINDMILL_POWER : 0);

            // ---------- Outtake at 4500 RPM ----------
            if (outtakeOn) {
                outtakeMotor.setVelocity(OUTTAKE_TICKS_PER_SEC);
            } else {
                outtakeMotor.setVelocity(0);
            }

            // ---------- Telemetry ----------
            double outtakeRpm = outtakeMotor.getVelocity() * 60.0 / OUTTAKE_TICKS_PER_REV;
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Windmill", windmillOn ? "ON" : "OFF");
            telemetry.addData("Outtake", outtakeOn ? "ON" : "OFF");
            telemetry.addData("Outtake RPM", "%.0f / %.0f", outtakeRpm, OUTTAKE_RPM);
            telemetry.update();
        }

        // Stop everything
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intakeMotor.setPower(0);
        outtakeMotor.setVelocity(0);
        rightIntakeServo.setPower(0);
        leftIntakeServo.setPower(0);
        windmillServo.setPower(0);
    }
}