////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, 2020 Cirrus Logic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Header File Preprocessor Directive
////////////////////////////////////////////////////////////////////////////////
#ifndef CS42L42_Android_Button_Detect_H
#define CS42L42_Android_Button_Detect_H

#define ASSERTED 	1
#define DEASSERTED 	0

typedef enum
{
	UNPLUGGED = 0,        // 0b0000
	HS_PLUGGED,           // 0b0001
	HEADSET,              // 0b0010
	HEADPHONE,            // 0b0011
	INVALID,              // 0b0100
	OPTICAL,              // 0b0101
	BUTTON_RELEASED,      // 0b0110
	BUTTON_A,             // 0b0111
	BUTTON_B,             // 0b1000
	BUTTON_C,             // 0b1001
	BUTTON_D              // 0b1010
} STATE; // Indicates current status

STATE CS42L42_state_machine(void);  // State machine to be called in main loop
void CS42L42_INTB_ISR(void);		// ISR to be called on rising and falling
                                   // edge of CS42L42 INTB pin

extern volatile unsigned char cs42l42_intb_flag; // Indicate the status of the
                                                 // CS42L42 INTB pin

#endif
