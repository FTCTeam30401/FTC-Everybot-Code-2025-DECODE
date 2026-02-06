package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoBlueShort", group="Robot")
public class AutoBlueShort extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private ElapsedTime runtime = new ElapsedTime();
    private double CATAPULT_OFF_POWER = 0;
    private double CATAPULT_UP_POWER = 1.0;
    private double CATAPULT_DOWN_POWER = -1.0;

    //private double CATAPULT_HOLD_POWER = 0.2;
    private DcMotor intake = null;
    private double INTAKE_IN_POWER = -1.0;
    private double INTAKE_OFF_POWER = 0.0;

    private enum CatapultModes {UP, DOWN, HOLD}

    static final double FAST_FORWARD_SPEED = 0.8;
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final double FORWARD_INTAKE_SPEED = 0.3;
    static final double OFF_SPEED = 0.0;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.


// brings catapult down +
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {

        }
// catapult up FIRE! +
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
        }

// Turns catapult motors off +
        catapult1.setPower(CATAPULT_OFF_POWER);
        catapult2.setPower(CATAPULT_OFF_POWER);

        leftDrive.setPower(TURN_SPEED);
        rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
        }

        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.25)) {

        }

        leftDrive.setPower(TURN_SPEED);
        rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.65)) {
        }

        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.25)) {
            leftDrive.setPower(OFF_SPEED);
            rightDrive.setPower(OFF_SPEED);
        }
    }
}