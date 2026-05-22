#pragma once
/* Zephyr Definitions */
#include <SPI.h>

#define I2C_SDA 20
#define I2C_SCL 21

#define JSPI_MISO 22
#define JSPI_MOSI 23
#define JSPI_SCK 24

#define JMISC1 25
#define JMISC2 26
#define JMISC3 27
#define JMISC4 28
#define JMISC5 29
#define JMISC6 30
#define JMISC7 31
#define JMISC8 32
#define JMISC9 33
#define JMISC10 34
#define JMISC11 35
#define JMISC12 36
#define JMISC13 37
#define JMISC14 38
#define JMISC15 39
#define JMISC16 40
#define JMISC17 41
#define JMISC18 42
#define JMISC19 43
#define JMISC20 44
#define JMISC21 45
#define JMISC22 46
#define JMISC23 47
#define JMISC24 48
#define JMISC25 49

#define JMISC_I2C_SCL 40
#define JMISC_I2C_SDA 42

#define LED3_R 50
#define LED3_G 51
#define LED3_B 52
#define LED4_R 53
#define LED4_G 54
#define LED4_B 55

/* Sketch definitions */
// Motors
constexpr uint8_t BL_EN = JMISC12;
constexpr uint8_t BL_IN1 = JMISC10;
constexpr uint8_t BL_IN2 = JMISC8;
constexpr uint8_t FR_IN2 = JMISC5;
constexpr uint8_t FR_IN1 = JMISC3;
// constexpr uint8_t FR_EN = JMISC1;
// constexpr uint8_t BR_EN = JMISC2;
constexpr uint8_t FR_EN = 6;
constexpr uint8_t BR_EN = 9;
constexpr uint8_t BR_IN1 = JMISC4;
constexpr uint8_t BR_IN2 = JMISC6;
constexpr uint8_t FL_IN2 = JMISC7;
constexpr uint8_t FL_IN1 = JMISC9;
constexpr uint8_t FL_EN = JMISC11;

// Feedback encoders
constexpr uint8_t BL_ENC_B = JMISC25;
constexpr uint8_t BL_ENC_A = JMISC23;
constexpr uint8_t BR_ENC_A = JMISC15;
constexpr uint8_t BR_ENC_B = JMISC17;
constexpr uint8_t FR_ENC_B = JMISC19;
constexpr uint8_t FR_ENC_A = JMISC21;
constexpr uint8_t FL_ENC_A = JMISC14;
constexpr uint8_t FL_ENC_B = JMISC13;

// RC receiver channels — pins 2-9, wired H→A (pin 2 = CH_H, pin 9 = CH_A)
// constexpr uint8_t RC_PIN_A = 9;
// constexpr uint8_t RC_PIN_B = 8;
constexpr uint8_t RC_PIN_A = 12;
constexpr uint8_t RC_PIN_B = 11;
// constexpr uint8_t RC_PIN_C = 7;
// constexpr uint8_t RC_PIN_D = 6;
constexpr uint8_t RC_PIN_C = 8;
constexpr uint8_t RC_PIN_D = 7;
constexpr uint8_t RC_PIN_E = 5;
constexpr uint8_t RC_PIN_F = 4;
constexpr uint8_t RC_PIN_G = 3;
constexpr uint8_t RC_PIN_H = 2;

// Initialization of sketch pins
inline void pinSetup() {
    pinMode(FL_EN, OUTPUT);
    pinMode(FL_IN1, OUTPUT);
    pinMode(FL_IN2, OUTPUT);
    pinMode(BL_EN, OUTPUT);
    pinMode(BL_IN1, OUTPUT);
    pinMode(BL_IN2, OUTPUT);
    pinMode(BR_EN, OUTPUT);
    pinMode(BR_IN1, OUTPUT);
    pinMode(BR_IN2, OUTPUT);
    pinMode(FR_EN, OUTPUT);
    pinMode(FR_IN1, OUTPUT);
    pinMode(FR_IN2, OUTPUT);

    // Encoder pins
    pinMode(FL_ENC_A, INPUT);
    pinMode(FL_ENC_B, INPUT);
    pinMode(BL_ENC_A, INPUT);
    pinMode(BL_ENC_B, INPUT);
    pinMode(FR_ENC_A, INPUT);
    pinMode(FR_ENC_B, INPUT);
    pinMode(BR_ENC_A, INPUT);
    pinMode(BR_ENC_B, INPUT);

    // RC receiver pins
    pinMode(RC_PIN_A, INPUT);
    pinMode(RC_PIN_B, INPUT);
    pinMode(RC_PIN_C, INPUT);
    pinMode(RC_PIN_D, INPUT);
    pinMode(RC_PIN_E, INPUT);
    pinMode(RC_PIN_F, INPUT);
    pinMode(RC_PIN_G, INPUT);
    pinMode(RC_PIN_H, INPUT);
}