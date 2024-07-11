/*
  Author: jehongjeon27
  version: 1.1
*/

#include "MICS_4514.h"
#include "Arduino.h"

//Constructor
MICS_4514::MICS_4514(uint8_t pin_red, uint8_t pin_nox, uint8_t pin_pre) {

  _pin_red = pin_red;
  _pin_nox = pin_nox;
  _pin_pre = pin_pre;

  pinMode(_pin_red, INPUT);
  pinMode(_pin_nox, INPUT);
  pinMode(_pin_pre, OUTPUT);

  digitalWrite(_pin_pre, LOW);
}

void MICS_4514::setWarmupTime(uint32_t time) {
  _warmup_time = time;
}

void MICS_4514::warmupStart() {
  _warmup_started = true;
  _warmup_start_time = millis();
  digitalWrite(_pin_pre, HIGH);
  _heating_status = true;
}


bool MICS_4514::sensorReady() {
  if (_warmup_started) {
    if (_warmup_finished) {
      return true;
    } else {
      if (millis() - _warmup_start_time < _warmup_time) {
        return false;
      } else {
        _warmup_finished = true;
        return true;
      }
    }

  } else {
    return false;
  }
}


float MICS_4514::read_R_RED() {
  uint16_t adc_red = analogRead(_pin_red);
  float vout_red = adc_red * ( (float)V_MCU_VCC / (float)MCU_ADC_MAX_VAL);  // MCU Voltage = 5v | MCU ADC bit = 10bit (0 ~ 1023)
                                                                            // This value may be changed by your MCU!!
  return (V_SENSOR_VCC / vout_red - 1) * R_LOAD_RED;
}

float MICS_4514::read_R_OX() {
  uint16_t adc_ox  = analogRead(_pin_nox);
  float vout_ox  = adc_ox  * ( (float)V_MCU_VCC / (float)MCU_ADC_MAX_VAL);  // MCU Voltage = 5v | MCU ADC bit = 10bit (0 ~ 1023)
                                        // This value may be changed by your MCU!!
  return (V_SENSOR_VCC / vout_ox  - 1) * R_LOAD_OX ;
}

void MICS_4514::setR0() {
  _r0_red = read_R_RED();
  _r0_ox = read_R_OX();
}



bool MICS_4514::getHeatingState() {
  return _heating_status;
}

void MICS_4514::setHeatingState(uint8_t on_off) {
  digitalWrite(_pin_pre, on_off);
  _heating_status = true;
}










float MICS_4514::getCarbonMonoxide() {
  float rs_r0 = read_R_RED() / _r0_red;
  float gas_ppm;

  if (rs_r0 < 0.01) {
    // value is too small
    return -1;

  } else if (rs_r0 > 4) {
    // value is too big
    return -1;

  } else {
    // y = A * (data ^ B)
    gas_ppm =  4.422139 * pow(rs_r0, -1.177184);

    if (gas_ppm < 1) {
      return 1;
    } else if (gas_ppm > 1000) {
      return 1000;
    } else {
      return gas_ppm;
    }
  }

  return -1;
}

float MICS_4514::getNitrogenDioxide() {
  float rs_r0 = read_R_OX() / _r0_ox;
  float gas_ppm;

  if (rs_r0 < 0.3) {
    // value is too small
    return -1;

  } else if (rs_r0 > 70) {
    // value is too big
    return -1;

  } else {
    // y = A * (data ^ B)
    gas_ppm =  0.15 * pow(rs_r0, 1);

    if (gas_ppm < 0.05) {
      return 0.05;
    } else if (gas_ppm > 10) {
      return 10;
    } else {
      return gas_ppm;
    }
  }

  return -1;
}

float MICS_4514::getEthanol() {
  float rs_r0 = read_R_RED() / _r0_red;
  float gas_ppm;

  if (rs_r0 < 0.3) {
    // value is too small
    return -1;

  } else if (rs_r0 > 0.025) {
    // value is too big
    return -1;

  } else {
    // y = A * (data ^ B)
    gas_ppm =  1.543142 * pow(rs_r0, -1.568613);

    if (gas_ppm < 10) {
      return 10;
    } else if (gas_ppm > 500) {
      return 500;
    } else {
      return gas_ppm;
    }
  }

  return -1;
}

float MICS_4514::getHydrogen() {
  float rs_r0 = read_R_RED() / _r0_red;
  float gas_ppm;

  if (rs_r0 < 0.017) {
    // value is too small
    return -1;

  } else if (rs_r0 > 0.9) {
    // value is too big
    return -1;

  } else {
    // y = A * (data ^ B)
    gas_ppm =  0.846238 * pow(rs_r0, -1.754911);

    if (gas_ppm < 1) {
      return 1;
    } else if (gas_ppm > 1000) {
      return 1000;
    } else {
      return gas_ppm;
    }
  }

  return -1;
}

float MICS_4514::getAmmonia() {
  float rs_r0 = read_R_RED() / _r0_red;
  float gas_ppm;

  if (rs_r0 < 0.252) {
    // value is too small
    return -1;

  } else if (rs_r0 > 1) {
    // value is too big
    return -1;

  } else {
    // y = A * (data ^ B)
    gas_ppm =  1.000000 * pow(rs_r0, -4.507576);

    if (gas_ppm < 1) {
      return 1;
    } else if (gas_ppm > 500) {
      return 500;
    } else {
      return gas_ppm;
    }
  }

  return -1;
}

float MICS_4514::getMethane() {
  float rs_r0 = read_R_RED() / _r0_red;
  float gas_ppm;

  if (rs_r0 < 0.01) {
    // value is too small
    return -1;

  } else if (rs_r0 > 0.715) {
    // value is too big
    return -1;

  } else {
    // y = A * (data ^ B)
    gas_ppm =  719.230051 * pow(rs_r0, -4.1851127);

    if (gas_ppm < 1000) {
      return 1000;
    } else if (gas_ppm > 100000) {
      return 100000;
    } else {
      return gas_ppm;
    }
  }

  return -1;
}
