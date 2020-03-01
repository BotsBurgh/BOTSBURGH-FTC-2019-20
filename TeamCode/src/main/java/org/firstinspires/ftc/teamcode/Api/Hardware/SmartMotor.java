/*
 * Copyright 2020 FIRST Tech Challenge Team 11792
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Inspiration from Out of the Box FTC Team 7244
 * (https://github.com/OutoftheBoxFTC/SkyStone/blob/EeshwarTesting/TeamCode/src/main/java/HardwareSystems/HardwareDevices/SmartMotor.java)
 */

package org.firstinspires.ftc.teamcode.Api.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import lombok.Getter;
import lombok.Setter;

public class SmartMotor {
    @Getter private DcMotor motor;
    @Getter private double power;
    private double oldPower;

    // Motor configuration
    @Getter @Setter double maxPower = 1;

    public SmartMotor(DcMotor motor) {
        this.motor = motor;
        power = 0;
        oldPower = 0;
    }

    public void setPower(double power){
        if (Math.abs(power - oldPower) > 0) {
            oldPower = power;
            this.power = power;
            motor.setPower(power*maxPower);
        }
    }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void setTargetPosition(int targetPosition) {
        motor.setTargetPosition(targetPosition);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }
}
