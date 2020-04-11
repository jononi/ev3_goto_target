def button_enter_pressed():
    next_step = True
    brick.button_enter.onEvent(ButtonEvent.PRESSED, button_enter_pressed)
    next_step = False
    drive_train.driver_bc.setWheelRadius(3)
    drive_train.driver_bc.setBaseLength(13)
    drive_train.driver_bc.setInvertedDirection(True)
    drive_train.driver_bc.setTimestamp(control.millis())
    drive_train.driver_bc.setPose(0, 0, 90)
    console.send_to_screen()

def on_forever():
    if next_step:
        motors_speed = drive_train.driver_bc.driveTo(1.41, 1.41)
        console.log_value("w_r", Math.round_with_precision(motors_speed[0], 2))
        console.log_value("w_r", Math.round_with_precision(motors_speed[1], 2))
        motors.large_bc.stop()
        drive_train.driver_bc.predictPose(motors_speed[0], motors_speed[1], control.millis())
        drive_train.driver_bc.setTimestamp(control.millis())
        currentPose = drive_train.driver_bc.getPose()
        console.log_value("x", Math.round_with_precision(currentPose[0], 2))
        console.log_value("y", Math.round_with_precision(currentPose[1], 2))
        console.log_value("theta", Math.round_with_precision(currentPose[2], 2))
        next_step = False

forever(on_forever)