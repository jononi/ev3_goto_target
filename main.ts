forever(function on_forever() {
    let motors_speed: number[];
    let currentPose: number[];
    let next_step: boolean;
    if (next_step) {
        motors_speed = drive_train.driverBC.driveTo(1.41, 1.41)
        console.logValue("w_r", Math.roundWithPrecision(motors_speed[0], 2))
        console.logValue("w_r", Math.roundWithPrecision(motors_speed[1], 2))
        motors.largeBC.stop()
        drive_train.driverBC.predictPose(motors_speed[0], motors_speed[1], control.millis())
        drive_train.driverBC.setTimestamp(control.millis())
        currentPose = drive_train.driverBC.getPose()
        console.logValue("x", Math.roundWithPrecision(currentPose[0], 2))
        console.logValue("y", Math.roundWithPrecision(currentPose[1], 2))
        console.logValue("theta", Math.roundWithPrecision(currentPose[2], 2))
        next_step = false
    }
    
})
