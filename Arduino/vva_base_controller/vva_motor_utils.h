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


// Motor pins
#define LEFT_BACK_PIN_1     3
#define LEFT_BACK_PIN_2     2
#define LEFT_FRONT_PIN_1    10
#define LEFT_FRONT_PIN_2    5
#define RIGHT_FRONT_PIN_1   6
#define RIGHT_FRONT_PIN_2   7
#define RIGHT_BACK_PIN_1    9
#define RIGHT_BACK_PIN_2    8


#define LEFT_FRONT_WHEEL    1
#define LEFT_BACK_WHEEL     2
#define RIGHT_FRONT_WHEEL   3
#define RIGHT_BACK_WHEEL    4


// Motor calibration parameters
int _motor_cmd_max = 255;
int _motor_cmd_min;
float _motor_max_angular_vel;
float _motor_min_angular_vel;


/********************************************************
 * Convert encoder ticks to radians
 */
float enc_2_rads(int enc_ticks);

/********************************************************
 * Init motor pins
 */
void init_motor_pins();

/********************************************************
 * Mapping angular velocity targets to motor commands
 */
int angularvel_2_motorcmd(float angular_vel);

/********************************************************
 * Move motor
 */
void move_motor(int wheel, int motor_command);
