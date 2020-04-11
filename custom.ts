/**
 * A differential drive robot extension
 * NOTE: largeBC => left motor on B, right motor on C
 */
//% color="#00751B" weight=89 icon="\uf10d"
namespace drive_train {
    enum MyEnum {
        //% block="one"
        One,
        //% block="two"
        Two
    }
    /**
     * A differential drive robot
     */
    //% fixedInstances
    export class Driver {
        motors: motors.SynchedMotorPair;
        wheelRadius: number;
        baseLength: number;
        invertedDirection: boolean;
        maxw: number;
        private robotPose: Pose;
        private timestamp: number;

        constructor(motors: motors.SynchedMotorPair) {
            this.motors = motors;
            this.wheelRadius = 3; // cm
            this.baseLength = 12; // cm
            this.maxw = 170 / 60 * 2 * Math.PI; // rad / s
            this.invertedDirection = false
            this.timestamp = 0
        }

        //% group="Move"
        //% blockId=motorDrive block="drive %drive_train to location: %target_x m %target_y m"
        //% inlineInputMode=inline
        //% weight=95 blockGap=8
        driveTo(target_x: number, target_y: number) {
            let Kv = 1.0  // translation speed constant
            let Kw = 1.0  // rotation speed constant
            let V = 0  // translation speed
            let W = 0  // rotation speed
            let motorSpeed = [0, 0]
            let deltaX = target_x - this.robotPose.x  //meters
            let deltaY = target_y - this.robotPose.y
            // updating bearing and distance to target
            let bearing = Math.atan2(deltaY, deltaX) - this.robotPose.theta  // !radians!
            let distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2))  // !meters!

            console.log("distance to: " + `${Math.roundWithPrecision(distance, 2)}m`)
            console.log("bearing to: " + `${Math.roundWithPrecision(radToDeg(bearing), 2)}°`)

            if (distance > 0.05) {
                // using Lyapunov function for stable controls
                V = Kv * distance * Math.cos(bearing)  // m/sec
                W = Kv * Math.sin(bearing) * Math.cos(bearing) + Kw * bearing // rad /sec
            } else {
                // target reached, stop motors
                V = 0
                W = 0
            }

            if (distance < 0.2 && Math.abs(bearing) > 90) {
                // target was overpassed, spin around
                V = 0
                W = Kv * Math.sin(bearing) * Math.cos(bearing) + Kw * bearing
            }

            motorSpeed[0] = (2 * V + W * this.baseLength) / (2 * this.wheelRadius) // rad / sec
            motorSpeed[1] = (2 * V - W * this.baseLength) / (2 * this.wheelRadius) // rad / sec

            const pwm_r = Math.clamp(0, 90, motorSpeed[0] / this.maxw * 100) // %
            // const pwm_l = Math.clamp(0, 90, Math.round(motorSpeed[1] / this.maxw * 100))
            const pwm_l = Math.clamp(0, 90, motorSpeed[1] / this.maxw * 100) // %

            console.log("L: " + `${pwm_l}%` + " R: " + `${pwm_r}%`)


            // turn on drive motors at calculated speed/power
            if (this.invertedDirection) {
                this.motors.tank(-1 * pwm_r, -1 * pwm_l)
            } else {
                this.motors.tank(pwm_r, pwm_l)
            }

            return motorSpeed
        }

        //% group="Estimate"
        //% blockId=predictPose block="predict %drive_train pose with controls %control_w_r %control_w_l at %time(ms)"
        //% inlineInputMode=inline
        //% weight=95 blockGap=8
        predictPose(control_w_r: number, control_w_l: number, time: number) {
            let deltaT = (time - this.timestamp) / 1000 // convert ms to sec to keep everything SI
            // pose a priori estimate
            let predictedD = this.wheelRadius * (control_w_r + control_w_l) * deltaT / 2
            let predictedDeltaTheta = this.wheelRadius * (control_w_r - control_w_l) * deltaT / this.baseLength
            this.robotPose.x = this.robotPose.x + predictedD * Math.cos(this.robotPose.theta + predictedDeltaTheta / 2)
            this.robotPose.y = this.robotPose.y + predictedD * Math.sin(this.robotPose.theta + predictedDeltaTheta / 2)
            this.robotPose.theta = this.robotPose.theta + predictedDeltaTheta
            // error covariance matrix a priori estimate
            // TODO
        }

        /**
         * Sets the wheel radius in centimeters
         * @param cm the radios of a wheel in cm, eg: 3
         */
        //% blockId=chassisSetWheelRadius block="set %drive_train|wheel radius to %cm (cm)"
        //% group="Properties"
        setWheelRadius(cm: number) {
            this.wheelRadius = cm;
        }

        /**
         * Sets the base length in centimeters
         * @param cm the base length in cm, eg: 12
         */
        //% blockId=chassisSetBaseLength block="set %drive_train base length to %cm (cm)"
        //% group="Properties"
        setBaseLength(cm: number) {
            this.baseLength = cm;
        }

        //% block="set %drive_train motors|inverted %value"
        //% group="Properties"
        setInvertedDirection(value: boolean) {
            this.invertedDirection = value;
        }

        //% block="save %drive_train timestamp %ts"
        //% group="Properties"
        setTimestamp(ts: number) {
            this.timestamp = ts
        }

        //% group="Estimate"
        //% block="set %drive_train pose to %x (m) %y (m) %theta (degree)"
        //% inlineInputMode=inline
        setPose(x: number, y: number, theta: number) {
            this.robotPose = new Pose(x, y, degToRad(theta))
        }

        //% group="Estimate"
        //% block="get %drive_train current pose estimate"
        getPose(): number[] {
            return [this.robotPose.x, this.robotPose.y, radToDeg(this.robotPose.theta)]
        }

        toString(): string {
            return `chassis ${this.motors ? this.motors.toString() : "--"} base ${this.baseLength}, wheel ${this.wheelRadius}`;
        }
    }

    //% blockId=degreeToRadian
    //% block = "set %degree to radian"
    //% group="Utilities"
    export function degToRad(degree: number): number {
        return (degree * Math.PI) / 180
    }

    //% blockId=radianToDegree
    //%block="set %radian to degree"
    //% group="Utilities"
    export function radToDeg(radian: number): number {
        return (radian * 180) / Math.PI
    }

    class Pose {
        x: number
        y: number
        theta: number
        // covMtrx: mat3

        constructor(x: number, y: number, theta: number) {
            this.x = x
            this.y = y
            this.theta = theta
            // this.covMtrx = new mat3([0.025, 0, 0, 0, 0.025, 0, 0, 0, 0, 0.08])
        }
    }

    /*
    interface StateEstimate {
        val: Pose
        covE: Array<Array<number>>
    }*/

    //% fixedInstance whenUsed
    export const driverBC = new Driver(motors.largeBC);
}

