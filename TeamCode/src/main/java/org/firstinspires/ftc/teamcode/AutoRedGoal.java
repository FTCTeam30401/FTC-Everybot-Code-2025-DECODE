package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



import android.view.animation.GridLayoutAnimationController;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="AutoRedGoal", group="Robot")
public class AutoRedGoal extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private ElapsedTime runtime = new ElapsedTime();

    private double CATAPULT_UP_POWER = 1.0;
    private double CATAPULT_DOWN_POWER = -1.0;

    private double CATAPULT_OFF_POWER = 0;

    //private double CATAPULT_HOLD_POWER = 0.2;
    private DcMotor intake = null;
    private double INTAKE_IN_POWER = -1.0;
    private double INTAKE_OFF_POWER = 0.0;

    private enum CatapultModes {UP, DOWN, OFF}


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double FORWARD_INTAKE_SPEED = 0.4;
    static final double OFF_SPEED = 0.0;
    static final double SLOW_SPEED = 0.4;

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


        leftDrive.setPower(-TURN_SPEED);
        rightDrive.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
        }
// drives backward +
            leftDrive.setPower(-FORWARD_SPEED);
            rightDrive.setPower(-FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.7)) {

            }
            // turns right to face forward +
            leftDrive.setPower(TURN_SPEED);
            rightDrive.setPower(-TURN_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.35)) {

            }
// goes forward and starts intake +
            leftDrive.setPower(FORWARD_SPEED);
            rightDrive.setPower(FORWARD_SPEED);
            intake.setPower(INTAKE_IN_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.75)) {

            }

            // turns left and brings catapult down. Ready to intake
            leftDrive.setPower(-TURN_SPEED);
            rightDrive.setPower(TURN_SPEED);
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.20)) {
            }


            // goes forward and intakes 3 artifacts
            leftDrive.setPower(FORWARD_INTAKE_SPEED);
            rightDrive.setPower(FORWARD_INTAKE_SPEED);
            intake.setPower(INTAKE_IN_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 4)) {
            }

            // goes backwards. Ready to go back to the goal
            leftDrive.setPower(-FORWARD_SPEED);
            rightDrive.setPower(-FORWARD_SPEED);
            intake.setPower(INTAKE_IN_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {

            }

            // turns left to align
            leftDrive.setPower(-TURN_SPEED);
            rightDrive.setPower(TURN_SPEED);
            intake.setPower(INTAKE_OFF_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.85)) {
            }

            // goes forward and goes to goal to shoot
            leftDrive.setPower(FORWARD_SPEED);
            rightDrive.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.5)) {

            }
            leftDrive.setPower(OFF_SPEED);
            rightDrive.setPower(OFF_SPEED);

            // catapult up FIRE!
            catapult1.setPower(CATAPULT_UP_POWER);
            catapult2.setPower(CATAPULT_UP_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            }

// Turns catapult motors off
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

            sleep(2000);

            // faster forward speed
            leftDrive.setPower(SLOW_SPEED);
            rightDrive.setPower(SLOW_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 4)) {
            }
// brings catapult down ready to intake
            leftDrive.setPower(-TURN_SPEED);
            rightDrive.setPower(TURN_SPEED);
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.07)) {
            }

            // goes forward and intakes 3 artifacts
            leftDrive.setPower(FORWARD_INTAKE_SPEED);
            rightDrive.setPower(FORWARD_INTAKE_SPEED);
            intake.setPower(INTAKE_IN_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 4)) {
            }
// goes backward
            leftDrive.setPower(-FORWARD_SPEED);
            rightDrive.setPower(-FORWARD_SPEED);
            intake.setPower(INTAKE_IN_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            }

            // turns left to align
            leftDrive.setPower(-TURN_SPEED);
            rightDrive.setPower(TURN_SPEED);
            intake.setPower(INTAKE_OFF_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.65)) {
            }


            //stop
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }
