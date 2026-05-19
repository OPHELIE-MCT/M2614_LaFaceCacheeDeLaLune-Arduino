#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>

#include "PINS.h"
#include "driver/FeedbackEncoder.h"
#include "driver/MecanumDriver.h"
#include "driver/RemoteController.h"
#include "driver/SpeedController.h"

MotorPinConfig frontLeftPins = {FL_IN1, FL_IN2, FL_EN};
MotorPinConfig frontRightPins = {FR_IN1, FR_IN2, FR_EN};
MotorPinConfig rearLeftPins = {BL_IN1, BL_IN2, BL_EN};
MotorPinConfig rearRightPins = {BR_IN1, BR_IN2, BR_EN};
MecanumPins mecanumPins = {frontLeftPins, frontRightPins, rearLeftPins, rearRightPins};
MecanumDriver mecanumDriver = MecanumDriver(mecanumPins);

RCReceiverPins rcPins = {RC_PIN_A, RC_PIN_B, RC_PIN_C, RC_PIN_D, RC_PIN_E, RC_PIN_F, RC_PIN_G, RC_PIN_H};
RemoteController rc = RemoteController(rcPins);

EncoderChannelPins frontLeftEncoderPins = {FL_ENC_A, FL_ENC_B};
EncoderChannelPins frontRightEncoderPins = {FR_ENC_A, FR_ENC_B};
EncoderChannelPins rearLeftEncoderPins = {BL_ENC_A, BL_ENC_B};
EncoderChannelPins rearRightEncoderPins = {BR_ENC_A, BR_ENC_B};
FeedbackEncoderPins encoderPins = {
    frontLeftEncoderPins,
    frontRightEncoderPins,
    rearLeftEncoderPins,
    rearRightEncoderPins,
};
FeedbackEncoder encoders = FeedbackEncoder(encoderPins);

SpeedControllerConfig speedControllerConfig = {
    .kp = 0.35f,
    .ki = 0.08f,
    .kd = 0.02f,
    .integralMin = -400.0f,
    .integralMax = 400.0f,
    .outputMin = -500.0f,
    .outputMax = 500.0f,
};

// PSpeedController selectedSpeedControllerImpl(speedControllerConfig);
// PISpeedController selectedSpeedControllerImpl(speedControllerConfig);
PIDSpeedController selectedSpeedControllerImpl(speedControllerConfig);
SpeedController& speedController = selectedSpeedControllerImpl;

void setup() {
    Monitor.begin();
    Monitor.println("Starting up...");
    pinSetup();
    mecanumDriver.begin();
    rc.begin();
    encoders.begin();
}

void loop() {}