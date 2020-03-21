/*
 * Networking
 */

typedef union {
    int32_t i[4];
    float f[4];
} sdp_data;

struct sdp_single { // Sensor Data Point
  uint8_t id;
  sdp_data data;
};

struct parameter_change {
    int32_t par_id;
    union {
      int32_t i;
      float f;
    } new_val;
    int32_t ack_p; // is this a confirmation?
    /*
     * ack_p = 0 -> a package send by the utility towards the controller
     * ack_p = 1 -> the controller acknowledging a change
     *   (same val and id as the request)
     * ack_p = 2 -> error_val, request contains a val outside
     *   the valid range for given par_id
     * ack_p = 3 -> error_id, there is no parameter with the given par_id
     * ack_p = 4 -> echo the currrent value
     */
};

#define PARAMETER_COUNT 64
#define PARAMETER_PORT "6355"
#define SINK_PORT "6354"
#define SINK_ADDR "192.168.0.2"
#define MAX_PER_DATAGRAM 128

#define CONTROLLER_ADDR "192.168.0.1"
#define SECRET_SSID "skate_control"
#define SECRET_PASS "4567"

/*
 * Motor
 */
#define MOTOR_HIGH 1023
#define MOTOR_LOW 0
#define MOTOR_NEUTRAL 512

#define MOTOR_DIR_ACC 1;
#define MOTOR_DIR_BREAK -1;
#define MOTOR_DIR_NEUTRAL 0;

#define MOTOR_REACTION_TIME_ACC 1000
#define MOTOR_REACTION_TIME_BREAK 1000

/*
 * Sensors
 */

#define HUMAN_WEIGHT_THRESHOLD_FRONT 5000000 // todo
#define HUMAN_WEIGHT_THRESHOLD_BACK 5000000 // todo
#define SENSOR_ID_LC 0     // int   front, back, diff_smooth
#define SENSOR_ID_VEL 1    // int   cur, target TODO
#define SENSOR_ID_ACC 2    // float acc x, y, z
#define SENSOR_ID_GYRO 3   // float gyro x, y, z
#define SENSOR_ID_MOTOR_WINDOW 4    // int   midpoint, upper, lower, diff_scaled
/*
 * Delay
 */
#define DELAY_ADC_READ 2000 // delayMicroseconds()
