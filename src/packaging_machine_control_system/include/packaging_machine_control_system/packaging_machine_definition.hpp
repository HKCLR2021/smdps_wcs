#ifndef PACKAGING_MACHINE_DEFINITION_HPP_
#define PACKAGING_MACHINE_DEFINITION_HPP_

/*
  Caution:
  This header file is important
  Please read the parameters carefully before you change it
*/

#define DAYS          7
#define CELLS_PER_DAY 4
#define CELLS         28

#define PKG_PREFIX  5
#define PKG_POSTFIX 7

#define NO_OF_REED_SWITCHS          8
#define NO_OF_VALVES                4
#define NO_OF_PHOTOELECTRIC_SENSERS 8
#define NO_OF_PILL_GATES            4

#define MOTOR_ENABLE  1
#define MOTOR_DISABLE 0

#define PKG_DIS_RADIUS        18   // in mm unit
#define PKG_DIS_FEED_DIR      1
#define PKG_DIS_UNFEED_DIR    0    // should not be used
#define PKG_DIS_MARGIN_FACTOR 1.065

#define PILL_GATE_RADIUS              10   // in mm unit
#define PILL_GATE_WIDTH               44   // in mm unit
#define PILL_GATE_OPEN_DIR            1
#define PILL_GATE_CLOSE_DIR           0
#define PILL_GATE_CLOSE_MARGIN_FACTOR 1.05

#define PULSES_PER_REV 3200 // make sure this value is the same with the controller

#define CONVEYOR_FWD 1
#define CONVEYOR_REV 0      // should not be used
#define CONVEYOR_SPEED 250
#define SQUEEZER_SPEED 1100

#define HEATER_ON  1
#define HEATER_OFF 0

#define SQUEEZER_ACTION_PUSH 1
#define SQUEEZER_ACTION_PULL 0

#define MTRL_BOX_GATE_OPEN        1
#define MTRL_BOX_GATE_OPEN_STATE  1
#define MTRL_BOX_GATE_CLOSE       0
#define MTRL_BOX_GATE_CLOSE_STATE 0

#define STOPPER_PROTRUDE       1
#define STOPPER_PROTRUDE_STATE 0
#define STOPPER_SUNK           0
#define STOPPER_SUNK_STATE     1

#define DELAY_GENERAL_VALVE           2s    // General valve delay in seconds
#define DELAY_GENERAL_STEP            250ms // Step delay in milliseconds
#define DELAY_PKG_DIS_WAIT_PRINTER    200ms // Wait for printer delay in milliseconds
#define DELAY_PKG_DIS_BEFORE_SQUEEZER 100ms // Before squeezer delay in milliseconds
#define DELAY_SQUEEZER                500ms // Squeezer delay in milliseconds
#define DELAY_CONVEYOR_TESTING        1s    // Conveyor testing delay in seconds
#define DELAY_MTRL_BOX_GATE           1s    // material box gate in seconds
#define DELAY_CO                      50ms  // CANopen delay
#define DELAY_CO_L                    200ms // CANopen delay larger delay
#define DELAY_VALVE_WAIT_FOR          250ms // wait_for delay for valves
#define DELAY_MOTOR_WAIT_FOR          250ms // wait_for delay for motors
#define DELAY_ORDER_START_WAIT_FOR    1s    // wait_for delay for order start

#define MIN_TEMP 100

#endif  // PACKAGING_MACHINE_DEFINITION_HPP_