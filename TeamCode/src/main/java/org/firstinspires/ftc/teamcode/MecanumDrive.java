
/* Copyright (c) 2017 FIRST. All rights reserved.
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
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * CRServo Notes:
 * Document: https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/CRServo.html
 *
 * CRServo class is inherited from DCMotor Class
 */

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Drive Test", group="Iterative Opmode")
//@Disabled
public class MecanumDrive extends OpMode
{
    // Declare OpMode members.
    private double factor = 0.6;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor toprght = null;
    private DcMotor toplft = null;
    private DcMotor btmrght = null;
    private DcMotor btmlft = null;

    private CRServo DownSlider = null;
    private CRServo UpSlider = null;

    private Servo PlateMover = null;
    private Servo Finger = null;
    private Servo Rist = null;

    // Default Setup
    private double DEFAULT_PLATE_MOVER_POSITION = 0;
    private double DOWN_PLATE_MOVER_POSITION = 1;

    private double DEFAULT_SLIDE_POWER = 100;

    private boolean is_grapper_close = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        toprght  = hardwareMap.get(DcMotor.class, "toprght");
        toplft  = hardwareMap.get(DcMotor.class, "toplft");
        btmrght = hardwareMap.get(DcMotor.class, "btmrght");
        btmlft = hardwareMap.get(DcMotor.class, "btmlft");


        DownSlider = hardwareMap.get(CRServo.class, "downslide");
        UpSlider = hardwareMap.get(CRServo.class, "upslide");

        PlateMover = hardwareMap.get(Servo.class, "platemover");
        Finger = hardwareMap.get(Servo.class, "finger");
        Rist = hardwareMap.get(Servo.class, "rist");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        toplft.setDirection(DcMotor.Direction.FORWARD);
        btmlft.setDirection(DcMotor.Direction.FORWARD);
        toprght.setDirection(DcMotor.Direction.REVERSE);
        btmrght.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Set up Servos
        PlateMover.setDirection(Servo.Direction.FORWARD);
        PlateMover.setPosition(this.DEFAULT_PLATE_MOVER_POSITION);

        // One is close
        Finger.setPosition(0);
        Rist.setPosition(0.5);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double strafing = gamepad1.left_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y;
        // rightPower = -gamepad1.right_stick_y;

        // Send calculated power to wheels
        toplft.setPower(leftPower * factor);
        btmlft.setPower(leftPower * factor);
        toprght.setPower(rightPower * factor);
        btmrght.setPower(rightPower * factor);

        if (strafing < 0) {
            toplft.setPower(strafing);
            toprght.setPower(-strafing);
            btmlft.setPower(-strafing);
            btmrght.setPower(strafing);

        } if (strafing > 0) {
            toplft.setPower(strafing);
            toprght.setPower(-strafing);
            btmlft.setPower(-strafing);
            btmrght.setPower(strafing);
        }

        if (gamepad1.right_bumper && gamepad1.dpad_up) {
            this.liftArm(UpSlider, DcMotorSimple.Direction.REVERSE, this.DEFAULT_SLIDE_POWER);
        } else if (gamepad1.right_bumper && gamepad1.dpad_down) {
            this.liftArm(UpSlider, DcMotorSimple.Direction.FORWARD, this.DEFAULT_SLIDE_POWER);
        } else {
            this.stopLiftArm(UpSlider);
        }

        if(gamepad1.left_bumper && gamepad1.dpad_up){
            this.liftArm(DownSlider, DcMotorSimple.Direction.FORWARD,  this.DEFAULT_SLIDE_POWER);
        } else if (gamepad1.left_bumper && gamepad1.dpad_down){
            this.liftArm(DownSlider, DcMotorSimple.Direction.REVERSE, this.DEFAULT_SLIDE_POWER);
        } else {
            this.stopLiftArm(DownSlider);
        }

        if(gamepad1.y){
            PlateMover.setPosition(this.DEFAULT_PLATE_MOVER_POSITION);
        }
        if(gamepad1.a){
            PlateMover.setPosition(this.DOWN_PLATE_MOVER_POSITION);
        }

        if(gamepad1.x){
            if(Finger.getPosition() == 1){
                Finger.setPosition(0);
                try{Thread.sleep(500);}catch (Exception e){}
            }else if(Finger.getPosition() == 0){
                Finger.setPosition(1);
                try{Thread.sleep(500);}catch (Exception e){}
            }
        }

        if(gamepad1.b){
            Finger.setPosition(1);
        }

        if(gamepad1.dpad_left){
            Rist.setPosition(0.5);
        }
        if(gamepad1.dpad_right){
            // open
            Rist.setPosition(1);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Factor", "FactorValue (%.2f)", factor);
        telemetry.addData("Strafing", "StrafingValue (%.2f)", strafing);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        telemetry.addData("PlateMover", "Port (%d) | Position (%.2f)", PlateMover.getPortNumber(), PlateMover.getPosition());
        telemetry.addData("Finger", "Port (%d) | Position (%.2f)", Finger.getPortNumber(), Finger.getPosition());
        telemetry.addData("Rist", "Port (%d) | Position (%.2f)", Rist.getPortNumber(), Rist.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    // Need to test
    public void liftArm(CRServo servo, DcMotorSimple.Direction direction, double power) {
        servo.setDirection(direction);
        servo.setPower(power);
    }

    public void stopLiftArm(CRServo servo){
        servo.setPower(0);
    }
}
