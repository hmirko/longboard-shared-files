//#define DEBUG_MOTOR
//#define DEBUG_SENSOR
//#define DEBUG_WIFI

#include <ADS126X.h>
#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#include "common.h"
#include "pins.h"

#define MINMAX(l, h, v) max(l, min(h, v))


/* defines which event loop will run
   1 -> calibration loop
   2 -> driving loop
   3 -> active mode (driving mode, permanent breaking, no acceleration)
   4 -> passive mode (driving mode,  no acceleration / breaking)
*/
int32_t mode;

int32_t calibration_init_p;
int32_t calibration_diff_max;
int32_t calibration_diff_min;

#define CALIBRATION_TIME 20

/* ===== MOTOR_CONTROL ======
   Velocity is encoded as 10 bit value.
   The middle value is the zero movement position.
   Values from 512 towards 1023 encode forward movement,
   while values below 511 encode the intensity of breaking.
*/

#define VELOCITY_HISTORY_SIZE 16

int32_t diff_window_size = 200;

int32_t diff_window_midpoint;

uint16_t velocity_history[VELOCITY_HISTORY_SIZE];
uint16_t velocity_history_current;

int32_t preset_window_on;
int32_t preset_response_on;
int32_t preset_stepping_on;

/* coefficients for the response polynoms. One for acceleration and
   one for breaking */
float motor_response_acc_a;
float motor_response_acc_b;
float motor_response_acc_c;

float motor_response_brk_a;
float motor_response_brk_b;
float motor_response_brk_c;

int32_t motor_acc;
float motor_acc_weight;
int32_t motor_acc_last_change;

int32_t motor_full_throttle;
int32_t motor_full_break;
int32_t motor_gain;

/* ===== NETWORKING ===== */
IPAddress controller_addr;
IPAddress sink_addr;
WiFiUDP sinkUdp;
WiFiUDP parameterUdp;
char pass[] = SECRET_PASS;
char ssid[] = SECRET_SSID;
int ap_status = WL_IDLE_STATUS;
int parameter_port;

int sink_port;


/* ===== SENSORS ===== */
// positiv and negative reading pins for the 4 sensors
uint8_t pos_pin[4] =
{ 7, //ADS126X_AIN7;
  5, //ADS126X_AIN5;
  3, //ADS126X_AIN3;
  1 //ADS126X_AIN1;
};
uint8_t neg_pin[4] =
{ 6, //ADS126X_AIN6;
  4, //ADS126X_AIN4;
  2, //ADS126X_AIN2;
  0 //ADS126X_AIN0;
};

#define SENSOR_LC_HISTORY_SIZE 48
#define SENSOR_ACC_HISTORY_SIZE 8
#define SENSOR_GYRO_HISTORY_SIZE 8

int32_t sensor_front_offset;
int32_t sensor_back_offset;
int32_t sensor_back_history[SENSOR_LC_HISTORY_SIZE];
int32_t sensor_front_history[SENSOR_LC_HISTORY_SIZE];
int16_t sensor_lc_current;

float sensor_acc_x_history [SENSOR_ACC_HISTORY_SIZE];
float sensor_acc_y_history [SENSOR_ACC_HISTORY_SIZE];
float sensor_acc_z_history [SENSOR_ACC_HISTORY_SIZE];
int16_t sensor_acc_current;

float sensor_gyro_x_history [SENSOR_GYRO_HISTORY_SIZE];
float sensor_gyro_y_history [SENSOR_GYRO_HISTORY_SIZE];
float sensor_gyro_z_history [SENSOR_GYRO_HISTORY_SIZE];
int16_t sensor_gyro_current;

ADS126X adc;
uint16_t sensor_buf_usage;
struct sdp_single sensor_buf[MAX_PER_DATAGRAM];

/* ===== LED ===== */
int64_t led_last_change;

int led_green_state;
int led_red_state;
int led_yellow_state;

void setup() {
  int64_t now = millis();

  switch_mode(2); // we start in drive_mode
  Serial.begin(115200);
  //while (!Serial) { } // wait for serial port to connect. Needed for native USB port only
  Serial.println("Hello");

  // Sensor
  sensor_acc_current = 0;
  sensor_gyro_current = 0;
  sensor_buf_usage = 0;
  sensor_lc_current = 0;

  memset(sensor_buf, 0, sizeof(sdp_single) * MAX_PER_DATAGRAM);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  pinMode(PIN_LOAD_CELL_RESET, OUTPUT);
  pinMode(PIN_LOAD_CELL_START, OUTPUT);
	reset_init_ads();

  sensor_calibrate();

  // Motor
	switch_preset(0);

  analogReference(AR_DEFAULT);
  analogWriteResolution(10);

  velocity_history_current = 0;
  diff_window_midpoint = MOTOR_NEUTRAL;
  memset(&velocity_history, MOTOR_NEUTRAL, sizeof (uint16_t) * VELOCITY_HISTORY_SIZE);

  // Networking
  controller_addr.fromString(CONTROLLER_ADDR);
  parameter_port = atoi(PARAMETER_PORT);
  sink_addr.fromString(SINK_ADDR);
  sink_port = atoi(SINK_PORT);

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);
  }

  WiFi.config(controller_addr);

  ap_status = WiFi.beginAP(ssid);
  if (ap_status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  delay(1200);
  Serial.println("AP is online");

  sinkUdp.begin(sink_port);
  parameterUdp.begin(parameter_port);

  /* LED */

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);

  led_last_change = now;

  led_green_state = LOW;
  led_red_state = LOW;
  led_yellow_state = LOW;

  led_write_state();
}

void switch_preset(int new_preset) {
  int64_t now = millis();

	motor_full_throttle = 16000000;
	motor_full_break = 16000000;
	motor_gain = 2;

	motor_acc = 0;
	motor_acc_weight = 0.5;
	motor_acc_last_change = now;

  preset_stepping_on = 0;

  switch (new_preset) {
	case 0:
		motor_response_acc_a = 0.00253;
		motor_response_acc_b = -2.8884;
		motor_response_acc_c = 1326.82982;

		motor_response_brk_a = -0.00208;
		motor_response_brk_b = 2.066;
		motor_response_brk_c = 0;

		preset_window_on = 0;
		preset_response_on = 0;
		preset_stepping_on = 0;
		break;

	case 1:
		motor_response_acc_a = 0.00253;
		motor_response_acc_b = -2.8884;
		motor_response_acc_c = 1326.82982;

		motor_response_brk_a = -0.00208;
		motor_response_brk_b = 2.066;
		motor_response_brk_c = 0;

		preset_window_on = 1;
		preset_response_on = 0;
		preset_stepping_on = 0;
		break;

	case 2:
		motor_response_acc_a = 0.00253;
		motor_response_acc_b = -2.8884;
		motor_response_acc_c = 1326.82982;

		motor_response_brk_a = -0.00208;
		motor_response_brk_b = 2.066;
		motor_response_brk_c = 0;

		preset_window_on = 0;
		preset_response_on = 1;
		preset_stepping_on = 0;
		break;

	case 3:
		motor_response_acc_a = 0.00253;
		motor_response_acc_b = -2.8884;
		motor_response_acc_c = 1326.82982;

		motor_response_brk_a = -0.00208;
		motor_response_brk_b = 2.066;
		motor_response_brk_c = 0;

		preset_window_on = 1;
		preset_response_on = 1;
		preset_stepping_on = 0;
		break;

	default:
		break;
  }
}

void switch_mode(int new_mode) {
  switch (new_mode) {
    case 1:
      mode = 1;
      calibration_init_p = false;
      break;
    default:
      mode = new_mode;
      break;
  }
}

void loop() {
  switch (mode) {
    case 1:
      calibration_loop();
      break;
    case 2:
      driving_loop();
      break;
    case 3:
      active_loop();
      break;
    case 4:
      passive_loop();
      break;
  }
  wifi_action();
  led_action();
  delay(10);
}

void calibration_loop() {
  int64_t now = millis();

  if (!calibration_init_p) {

    Serial.println("starting calibration");

    calibration_init_p = true;
    calibration_diff_max = INT32_MIN;
    calibration_diff_min = INT32_MAX;
  }

  sensor_action();

  int32_t diff = sensor_get_difference();
  int32_t front = sensor_front_history[sensor_lc_current];
  int32_t back = sensor_back_history[sensor_lc_current];
  float front_grad = sensor_get_front_gradient();

  float front_grad_ratio = abs(front_grad) / front;

  /*
     only register new extreme values, if they are consistent over time (low ratio, not a peak)
     and there is a person on the board (the offsets represent the weight of the board itself)
  */
  if (front_grad_ratio < 0.005
      && (front > (sensor_front_offset + 4000000))
      && (back > (sensor_back_offset + 4000000)))
  {
    calibration_diff_max = max(diff, calibration_diff_max);
    calibration_diff_min = min(diff, calibration_diff_min);

    Serial.print("foudn new min/max -> [min ");
    Serial.print(calibration_diff_min);
    Serial.print("] [max ");
    Serial.print(calibration_diff_max);
    Serial.println("]");

  }

  analogWrite(A0, MOTOR_LOW);
}

void driving_loop() {
  sensor_action();
  motor_action();
  analogWrite(A0, velocity_history_peek());
  // safety_action();
}

void active_loop() {
  sensor_action();
  motor_action();
  analogWrite(A0, MOTOR_LOW);
  // safety_action();
}

void passive_loop() {
  sensor_action();
  motor_action();
  analogWrite(A0, MOTOR_NEUTRAL);
  //safety_action();
}

void safety_action() {
  int32_t front = sensor_front_history[sensor_lc_current];
  int32_t back = sensor_back_history[sensor_lc_current];

  if (front < (sensor_front_offset - sensor_front_offset * 0.1)
      || back < (sensor_back_offset - sensor_back_offset * 0.1)) {

    /* not measuring weight of the board itself -> turn of driving */
    // todo filter bumps
    mode = 3;
  } else {
    mode = 2;
  }

}

void led_action() {
  int64_t now = millis();

  switch (mode) {
    case 1:
      if ((now - 500) > led_last_change) {
        led_last_change = now;

        if (led_yellow_state == HIGH) {
          led_yellow_state = LOW;
        } else {
          led_yellow_state = HIGH;
        }
      }
      led_green_state = LOW;
      led_red_state = LOW;
      break;

    case 2:
      if ((now - 500) > led_last_change) {
        led_last_change = now;

        if (led_green_state == HIGH) {
          led_green_state = LOW;
        } else {
          led_green_state = HIGH;
        }

      }
      led_yellow_state = LOW;
      led_red_state = LOW;
      break;

    case 3:
      if ((now - 250) > led_last_change) {
        led_last_change = now;

        if (led_red_state == HIGH) {
          led_red_state = LOW;
        } else {
          led_red_state = HIGH;
        }
      }
      led_green_state = LOW;
      led_yellow_state = LOW;
      break;
  }
  led_write_state();
}

void wifi_action() {
  sinkUdp.beginPacket(sink_addr, sink_port);
  for (int p = 0; p < sensor_buf_usage; ++p) {
    sinkUdp.write((char*) &sensor_buf[p], sizeof (struct sdp_single));
  }
  sinkUdp.endPacket();

#ifdef DEBUG_WIFI
  Serial.print("datapoints sent -> ");
  Serial.println(sensor_buf_usage);
#endif

  sensor_buf_usage = 0;

  // Parameter changes
  struct parameter_change recv_buf = {0, 0, 0};
  int num_recv = parameterUdp.parsePacket();

  if (num_recv == sizeof(struct parameter_change)) {
#ifdef DEBUG_WIFI
    Serial.println("parameter change received");
#endif

    int bytes_read = parameterUdp.read((char*) &recv_buf, sizeof(struct parameter_change));

    if (bytes_read == sizeof(struct parameter_change)) {
      if (recv_buf.par_id < PARAMETER_COUNT &&
          recv_buf.par_id >= 0) {

        parameter_react(&recv_buf);
        recv_buf.ack_p = 1;

      } else {
        recv_buf.ack_p = 3;
      }

      parameterUdp.beginPacket(sink_addr, parameter_port);
      parameterUdp.write((char*) &recv_buf, sizeof(struct parameter_change));
      parameterUdp.endPacket();
    }
  }
  else {
#ifdef DEBUG_WIFI
    Serial.println("recv_parameter: num_recv != sizeof(struct parameter_change)");
    Serial.print("  expected -> ");
    Serial.println(sizeof(struct parameter_change));
    Serial.print("  received -> ");
    Serial.println(num_recv);
#endif
  }
}

void sensor_action() {
  sdp_data data;

  sensor_log_lc();
  sensor_log_acc();
  sensor_log_gyro();

  data.i[0] = sensor_front_history[sensor_lc_current];
  data.i[1] = sensor_back_history[sensor_lc_current];
  data.i[2] = sensor_get_difference();
  data.i[3] = velocity_history_peek();
  sensor_create_datapoint(SENSOR_ID_LC, &data);

  data.f[0] = sensor_acc_x_history[sensor_acc_current];
  data.f[1] = sensor_acc_y_history[sensor_acc_current];
  data.f[2] = sensor_acc_z_history[sensor_acc_current];
  sensor_create_datapoint(SENSOR_ID_ACC, &data);

  data.f[0] = sensor_gyro_x_history[sensor_acc_current];
  data.f[1] = sensor_gyro_y_history[sensor_acc_current];
  data.f[2] = sensor_gyro_z_history[sensor_acc_current];
  sensor_create_datapoint(SENSOR_ID_GYRO, &data);

}

void motor_action() {
  // normalize from raw sensor data to 0 to 1023 scale used for the motor

	int32_t target_v;

  int32_t diff_scaled = motor_scale_diff(sensor_get_difference());

	target_v = diff_scaled;

	if (preset_window_on == 1) {
		int32_t diff_distance_from_mid = abs(diff_scaled - diff_window_midpoint);
		// if current intention is outside of the window, shift it, else we do not react to achive smoothness
		if (diff_distance_from_mid > diff_window_size) {

			if (diff_scaled > diff_window_midpoint) {
				diff_window_midpoint = diff_window_midpoint + (diff_distance_from_mid - diff_window_size);

			} else {
				diff_window_midpoint = diff_window_midpoint - (diff_distance_from_mid - diff_window_size);
			}
		}
		/*
      if we are withing one window size of neutral, slowly creep to
      towards the midpoint. This prevents having a tiny bit of thrust
      when cruising in neutral.
		*/
		if ((diff_window_midpoint < (MOTOR_NEUTRAL + diff_window_size))
				&& (diff_window_midpoint > (MOTOR_NEUTRAL - diff_window_size))) {

			if (diff_window_midpoint > MOTOR_NEUTRAL) {
				diff_window_midpoint -= 2;
			} else {
				diff_window_midpoint += 2;
			}
		}

		target_v = diff_window_midpoint;

		sdp_data data;
		data.i[0] = diff_window_midpoint;
		data.i[1] = diff_window_midpoint - diff_window_size;
		data.i[2] = diff_window_midpoint + diff_window_size;
		data.i[3] = diff_scaled;
		sensor_create_datapoint(SENSOR_ID_MOTOR_WINDOW, &data);
	}

	if (preset_response_on == 1) {
		target_v = motor_map_diff_to_response(target_v);
	}

	if (preset_stepping_on == 1) {
		int32_t new_v;
		int32_t old_v = velocity_history_peek();
		int32_t target_diff = target_v - old_v;

		if (target_diff > 0) {
			if (motor_acc < -3) {
				motor_acc = motor_acc / (motor_acc * 1.4);
			} else {
				motor_acc = min(motor_acc + 2, 25);
			}
		} else if (target_diff < 0) {
			if (motor_acc > 3) {
				motor_acc = motor_acc / (motor_acc * 1.4);
			} else {
				motor_acc = max(motor_acc - 2, -25);
			}
		} else {
			motor_acc = 0;
		}

		new_v = old_v + motor_acc * motor_acc_weight;

		if (old_v < target_v) {
			target_v = MINMAX(MOTOR_LOW, target_v, new_v);
		} else {
			target_v = MINMAX(target_v, MOTOR_HIGH, new_v);
		}

	}

  velocity_history_push(target_v);


    sdp_data data;
    data.i[0] = target_v;
    data.i[1] = sensor_front_offset;
    data.i[2] = sensor_back_offset;
    data.i[3] = diff_scaled;
    sensor_create_datapoint(SENSOR_ID_VEL, &data);

}

int32_t sensor_read_front() {

	// dummy read to set the multiplexer
  adc.readADC1(pos_pin[PIN_LOAD_CELL_FRONT],
               neg_pin[PIN_LOAD_CELL_FRONT]);

	// wait for the multiplexer, else read garbage data
  delayMicroseconds(DELAY_ADC_READ);

	// actual read
  return adc.readADC1(pos_pin[PIN_LOAD_CELL_FRONT],
											neg_pin[PIN_LOAD_CELL_FRONT]);
}

int32_t sensor_read_back() {
  adc.readADC1(pos_pin[PIN_LOAD_CELL_BACK],
               neg_pin[PIN_LOAD_CELL_BACK]);

  delayMicroseconds(DELAY_ADC_READ);

  return adc.readADC1(pos_pin[PIN_LOAD_CELL_BACK],
                      neg_pin[PIN_LOAD_CELL_BACK]);
}

void sensor_log_lc() {
  int32_t front;
  int32_t back;


  front = sensor_read_front();

  delayMicroseconds(DELAY_ADC_READ); // todo is this required?

  back = sensor_read_back();

	int16_t prev = sensor_lc_current;
  sensor_lc_current = (sensor_lc_current + 1) % SENSOR_LC_HISTORY_SIZE;

  /* === sanity check ===
	 *
	 * the ads1262 seems to randomly switch into a state where only
	 * garbage data is read. The faulty value is known and can be
	 * caught. Since this value is inside the range of plausible
	 * measurements, repeat measurements are required.
	 *
	 * The ads1262 might randomly reset, which can be caught by reading
	 * a flag.
	 */

	if (front == 0x1000000
			&& back == 0x1000000
			&& adc.checkResetBit()) //detect if reset occured
	{
		reset_init_ads();

		sensor_front_history[sensor_lc_current] = sensor_front_history[prev];
		sensor_back_history[sensor_lc_current] = sensor_back_history[prev];
	}
	else
	{
		sensor_front_history[sensor_lc_current] = front;
		sensor_back_history[sensor_lc_current] = back;
	}
}

void sensor_log_acc() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    sensor_acc_current = (sensor_lc_current + 1) % SENSOR_ACC_HISTORY_SIZE;

    sensor_acc_x_history[sensor_acc_current] = x;
    sensor_acc_y_history[sensor_acc_current] = y;
    sensor_acc_z_history[sensor_acc_current] = z;
  }
}

void sensor_log_gyro() {
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    sensor_gyro_current = (sensor_lc_current + 1) % SENSOR_GYRO_HISTORY_SIZE;

    sensor_gyro_x_history[sensor_gyro_current] = x;
    sensor_gyro_y_history[sensor_gyro_current] = y;
    sensor_gyro_z_history[sensor_gyro_current] = z;
  }
}

int32_t sensor_get_back_smooth_decay() {
  int64_t sum_weights = (SENSOR_LC_HISTORY_SIZE * (SENSOR_LC_HISTORY_SIZE + 1)) / 2;
  sum_weights += 1;
  int64_t accumulator = 0;
  for (int i = 0; i < SENSOR_LC_HISTORY_SIZE; ++i) {
    accumulator += i * sensor_back_history[i];
  }

  return (int32_t) (accumulator / sum_weights);
}

int32_t sensor_get_front_smooth_decay() {
  int64_t sum_weights = (SENSOR_LC_HISTORY_SIZE * (SENSOR_LC_HISTORY_SIZE - 1)) / 2;

 // Serial.println((int) sum_weights);
  int64_t accumulator = 0;
  for (int i = 0; i < SENSOR_LC_HISTORY_SIZE; ++i) {
    accumulator += i * sensor_front_history[(i + sensor_lc_current) % SENSOR_LC_HISTORY_SIZE];
  }

  return (int32_t) (accumulator / sum_weights);
}

int32_t sensor_get_front_smooth() {
  int64_t accumulator = 0;
  for (int i = 0; i < SENSOR_LC_HISTORY_SIZE; ++i) {
    accumulator += sensor_front_history[i];
  }

  return (int32_t) (accumulator / SENSOR_LC_HISTORY_SIZE);
}

int32_t sensor_get_front_smooth_short() {
  int len = 16;
  int64_t accumulator = 0;
  for (int i = 0; i < len; ++i) {
    accumulator += sensor_front_history[(i + sensor_lc_current) % SENSOR_LC_HISTORY_SIZE];
  }

  return (int32_t) (accumulator / len);
}

int32_t sensor_get_back_smooth() {
  int64_t accumulator = 0;
  for (int i = 0; i < SENSOR_LC_HISTORY_SIZE; ++i) {
    accumulator += sensor_back_history[i];
  }

  return (int32_t) (accumulator / SENSOR_LC_HISTORY_SIZE);
}

float sensor_get_front_gradient() {
  int32_t oldest = sensor_front_history[(sensor_lc_current + 1) % SENSOR_LC_HISTORY_SIZE];
  int32_t newest = sensor_front_history[sensor_lc_current];

  int32_t diff = newest - oldest;
  return diff / SENSOR_LC_HISTORY_SIZE;
}

float sensor_get_back_gradient() {
  int32_t oldest = sensor_back_history[(sensor_lc_current + 1) % SENSOR_LC_HISTORY_SIZE];
  int32_t newest = sensor_back_history[sensor_lc_current];

  int32_t diff = newest - oldest;
  return diff / SENSOR_LC_HISTORY_SIZE;
}

int32_t sensor_get_difference() {
  return (sensor_get_front_smooth() - sensor_front_offset)
         - (sensor_get_back_smooth() - sensor_back_offset);
}

void sensor_calibrate() {
  sensor_init_history();

  sensor_front_4 = sensor_get_front_smooth();
  sensor_back_offset = sensor_get_back_smooth();

  //  Serial.print("sensor_calibrate() [front ");
  //  Serial.print(sensor_front_offset);
  //  Serial.print("] [back ");
  //  Serial.print(sensor_back_offset);
  //  Serial.println("]");
  //  Serial.print("front_raw ");
  //  Serial.print(sensor_front_history[sensor_lc_current]);
  //  Serial.print("\t offset ");
  //  Serial.println(sensor_front_offset);

}

void sensor_init_history() {
  for (int i = 0; i < SENSOR_LC_HISTORY_SIZE; ++i) {
    sensor_log_lc();
  }

  for (int i = 0; i < SENSOR_ACC_HISTORY_SIZE; ++i) {
    sensor_log_acc();
  }

  for (int i = 0; i < SENSOR_GYRO_HISTORY_SIZE; ++i) {
    sensor_log_gyro();
  }
}

void sensor_create_datapoint(uint8_t id, sdp_data *data) {
  if (sensor_buf_usage == (MAX_PER_DATAGRAM - 1)) {
    return;
  } else {
    memset(&sensor_buf[sensor_buf_usage],
           0, sizeof(sdp_single));

    sensor_buf[sensor_buf_usage].data.i[0] = data->i[0];
    sensor_buf[sensor_buf_usage].data.i[1] = data->i[1];
    sensor_buf[sensor_buf_usage].data.i[2] = data->i[2];
    sensor_buf[sensor_buf_usage].data.i[3] = data->i[3];
    sensor_buf[sensor_buf_usage].id = id;

    ++sensor_buf_usage;
  }
}


void velocity_history_push(int32_t v) {
  velocity_history_current = (velocity_history_current + 1) % VELOCITY_HISTORY_SIZE;
  velocity_history[velocity_history_current] = v;
}
int32_t velocity_history_peek() {
  return (int32_t) velocity_history[velocity_history_current];
}



int32_t motor_scale_diff(int32_t diff) {
  int32_t diff_scaled;
  int32_t scale = 1000; // scale so map does not break the int32_t boundaries by multiplication
  int32_t full_throttle = motor_full_throttle / scale;
  int32_t full_break = motor_full_break / scale;
  int32_t gain = motor_gain;
  int32_t lower_bound_in = -(full_throttle / gain);
  int32_t upper_bound_in = full_throttle / gain;

  int32_t lower_bound_out = MOTOR_LOW;
  if (preset_window_on == 1) {
    lower_bound_out -= diff_window_size;
  }

  int32_t upper_bound_out = MOTOR_HIGH;
  if (preset_window_on == 1) {
    upper_bound_out += diff_window_size;
  }


  diff_scaled = map(diff / scale, lower_bound_in, upper_bound_in, lower_bound_out, upper_bound_out);

  return MINMAX(lower_bound_out, upper_bound_out, diff_scaled);
}

/* Map from a real measured difference to a response curve.
  This enables us to react less agressive in the middle and more extreme towards the upper and lower bounds of the range */
int32_t motor_map_diff_to_response(int32_t diff) {
  float value;
  float a, b, c;
  float x = (float) diff;

  if (x < MOTOR_NEUTRAL) {
    a = motor_response_brk_a;
    b = motor_response_brk_b;
    c = motor_response_brk_c;
  } else {
    a = motor_response_acc_a;
    b = motor_response_acc_b;
    c = motor_response_acc_c;
  }

  value = a * x * x + b * x + c;

  if (x < MOTOR_NEUTRAL) {
    value = value = MINMAX(MOTOR_LOW, MOTOR_NEUTRAL, value);
  } else {
    value = value = MINMAX(MOTOR_NEUTRAL, MOTOR_HIGH, value);
  }

  return (int32_t) value;
}


void led_write_state() {
  digitalWrite(PIN_LED_GREEN, led_green_state);
  digitalWrite(PIN_LED_RED, led_red_state);
  digitalWrite(PIN_LED_YELLOW, led_yellow_state);
}

void parameter_react(struct parameter_change *request) {
  bool echo = false;
  int32_t id = request->par_id;

  if (request->ack_p == 4) {
    echo = true;
  }

  if (id < 0 || id >= PARAMETER_COUNT) {
    return;
  }

  switch (id) {
	case 0:
		if (!echo) {
			switch_mode(request->new_val.i);
		}
		request->new_val.i = mode;
		break;

	case 1:
		if (!echo) {
      motor_response_acc_a = request->new_val.f;
		}
		request->new_val.f = motor_response_acc_a;
		break;

	case 2:
		if (!echo) {
			motor_response_acc_b = request->new_val.f;
		}
		request->new_val.f = motor_response_acc_b;
		break;

	case 3:
		if (!echo) {
			motor_response_acc_c = request->new_val.f;
		}
		request->new_val.f = motor_response_acc_c;
		break;

	case 4:
		if (!echo) {
			motor_response_brk_a = request->new_val.f;
		}
		request->new_val.f = motor_response_brk_a;
		break;

	case 5:
		if (!echo) {
			motor_response_brk_b = request->new_val.f;
		}
		request->new_val.f = motor_response_brk_b;
		break;

	case 6:
		if (!echo) {
			motor_response_brk_c = request->new_val.f;
		}
		request->new_val.f = motor_response_brk_c;
		break;

	case 7:
		if (!echo) {
			diff_window_size = request->new_val.i;
		}
		request->new_val.i = diff_window_size;
		break;

		case 8:
		if (!echo) {
			motor_full_throttle = request->new_val.i;
		}
		request->new_val.i = motor_full_throttle;
		break;

  case 9:
		if (!echo) {
			motor_full_break = request->new_val.i;
		}
		request->new_val.i = motor_full_break;
		break;


	case 10:
		if (!echo) {
			motor_gain = request->new_val.i;
		}
		request->new_val.i = motor_gain;
		break;

	case 11:
		if (!echo) {
			switch_preset(request->new_val.i);
		}
		request->new_val.i = request->new_val.i;
		break;
  }
}

void reset_init_ads() {

  Serial.println("reset occured");
  digitalWrite(PIN_LOAD_CELL_RESET, LOW);
  delayMicroseconds(300);
  digitalWrite(PIN_LOAD_CELL_RESET, HIGH);


  digitalWrite(PIN_LOAD_CELL_START, HIGH);

  adc.setStartPin(PIN_LOAD_CELL_START);
  adc.begin(PIN_LOAD_CELL_CS);
  adc.setGain(ADS126X_GAIN_32);
  adc.setRate(ADS126X_RATE_7200);
  adc.setFilter(ADS126X_SINC4);
  adc.enableInternalReference();
  adc.startADC1(); // start conversion on ADC1
  adc.disableStatus();

  adc.disableCheck();
  adc.setDelay(ADS126X_DELAY_0);
  adc.clearResetBit();

	delayMicroseconds(500);
}
