/* 
 * Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
 * All rights reserved.
 * 
 * Released under the BSD License.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the <COPYRIGHT HOLDER>.
 * 4. Neither the name of the <COPYRIGHT HOLDER> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Created by:
 * andres.arboleda AT gmail DOT com, (oct/2020)
 */


/********************************************************
 * Convert encoder ticks to radians
 */
float enc_2_rads(long enc_ticks)
{
  // This encoder (Hall-effect CQ-Robot) produces 4424 ticks per revolution, so the formula to convert from ticks to rads is:
  // rads = (2*pi/4424)*ticks = 2*pi*ticks/4424

  float rads = 2 * PI * enc_ticks / 4424;
  return rads;
}


/********************************************************
 * Init motor pins
 */
void init_motor_pins() {
  
  // Change the PWM frequency.
  // Note: avoid to change TCCR0B (pins 4 and 13) becuase it would affect all the timing functions of the Arduino
  TCCR2B = TCCR2B & B11111000 | B00000001; // 31 KHz in ports 9 and 10   (other freq: 6:<20,   5:30.64, 4:122.55, 3:490.2,  2:3921.16, 1:31372.55)
  TCCR3B = TCCR3B & B11111000 | B00000001; // 31 KHz in ports 2, 3 and 5 (other freq: 6:<20,   5:30.64, 4:122.55, 3:490.2,  2:3921.16, 1:31372.55)
  TCCR4B = TCCR4B & B11111000 | B00000001; // 31 KHz in ports 6, 7 and 8 (other freq: 6:<20,   5:30.64, 4:122.55, 3:490.2,  2:3921.16, 1:31372.55)

  pinMode(LEFT_BACK_PIN_1,   OUTPUT);
  pinMode(LEFT_BACK_PIN_2,   OUTPUT);
  pinMode(LEFT_FRONT_PIN_1,  OUTPUT);
  pinMode(LEFT_FRONT_PIN_2,  OUTPUT);
  pinMode(RIGHT_FRONT_PIN_1, OUTPUT);
  pinMode(RIGHT_FRONT_PIN_2, OUTPUT);
  pinMode(RIGHT_BACK_PIN_1,  OUTPUT);
  pinMode(RIGHT_BACK_PIN_2,  OUTPUT);
}


/********************************************************
 * Mapping angular velocity targets to motor commands
 *   Note: motor commands are integers between 0 - 255
 *   It is assumed that motor commands are issued between motor_min_angular_vel and motor_max_angular_vel
 */
int angularvel_2_motorcmd(float angular_vel) {
  if (angular_vel == 0)
    return 0;
    
  float slope = (_motor_cmd_max - _motor_cmd_min) / (_motor_max_angular_vel - _motor_min_angular_vel);
  float intercept = _motor_cmd_max - slope * _motor_max_angular_vel;

  int motor_cmd = 0;

  if (angular_vel > 0) {
    motor_cmd = (int) (slope * angular_vel + intercept);
    if (motor_cmd > _motor_cmd_max) {
      motor_cmd = _motor_cmd_max;
    }
    if (motor_cmd < _motor_cmd_min) {
      motor_cmd = _motor_cmd_min;
    }
  }
  else {
    motor_cmd = (int) (slope * abs(angular_vel) + intercept);
    if (motor_cmd > _motor_cmd_max) {
      motor_cmd = _motor_cmd_max;
    }
    if (motor_cmd < _motor_cmd_min) {
      motor_cmd = _motor_cmd_min;
    }
    motor_cmd = -motor_cmd;
  }
  return motor_cmd;
}


/********************************************************
 * Move motor
 */
void move_motor(int wheel, int motor_command)
{
  switch (wheel) {
    case LEFT_FRONT_WHEEL:
      if (motor_command > 0) {
        analogWrite(LEFT_FRONT_PIN_1, motor_command);
        analogWrite(LEFT_FRONT_PIN_2, 0);
      }
      else if (motor_command < 0) {
        analogWrite(LEFT_FRONT_PIN_1, 0);
        analogWrite(LEFT_FRONT_PIN_2, -motor_command);
      }
      else {
        analogWrite(LEFT_FRONT_PIN_1, 0);
        analogWrite(LEFT_FRONT_PIN_2, 0);
      }
      break;
    case LEFT_BACK_WHEEL:
      if (motor_command > 0) {
        analogWrite(LEFT_BACK_PIN_1, motor_command);
        analogWrite(LEFT_BACK_PIN_2, 0);
      }
      else if (motor_command < 0) {
        analogWrite(LEFT_BACK_PIN_1, 0);
        analogWrite(LEFT_BACK_PIN_2, -motor_command);
      }
      else {
        analogWrite(LEFT_BACK_PIN_1, 0);
        analogWrite(LEFT_BACK_PIN_2, 0);
      }
      break;
    case RIGHT_FRONT_WHEEL:
      if (motor_command > 0) {
        analogWrite(RIGHT_FRONT_PIN_1, motor_command);
        analogWrite(RIGHT_FRONT_PIN_2, 0);
      }
      else if (motor_command < 0) {
        analogWrite(RIGHT_FRONT_PIN_1, 0);
        analogWrite(RIGHT_FRONT_PIN_2, -motor_command);
      }
      else {
        analogWrite(RIGHT_FRONT_PIN_1, 0);
        analogWrite(RIGHT_FRONT_PIN_2, 0);
      }
      break;
    case RIGHT_BACK_WHEEL:
      if (motor_command > 0) {
        analogWrite(RIGHT_BACK_PIN_1, motor_command);
        analogWrite(RIGHT_BACK_PIN_2, 0);
      }
      else if (motor_command < 0) {
        analogWrite(RIGHT_BACK_PIN_1, 0);
        analogWrite(RIGHT_BACK_PIN_2, -motor_command);
      }
      else {
        analogWrite(RIGHT_BACK_PIN_1, 0);
        analogWrite(RIGHT_BACK_PIN_2, 0);
      }
      break;
  }
}
