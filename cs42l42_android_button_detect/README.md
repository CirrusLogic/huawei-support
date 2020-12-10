CDB42L42 Android Button Detect
==============================
This code is offered as reference code with no support.
To integrate this code in to you project you will need to provide the following
functions:
void Delay_ms(unsigned int ms); - Function to delay by a set number of
                                  milliseconds where the  parameter ms is the
                                  number of milliseconds.
void Delay_us(unsigned int us); - Function to delay by a set number of
                                  microseconds where the parameter us is the
                                  number of microseconds.
void RegWrite (unsigned char reg_addr, unsigned char reg_value,
              unsigned char device_addr); - Function to write to a register on
                                            the device where reg_addr is the
                                            register address, reg_value is the
                                            value to set the register to and
                                            device_addr is the address of the
                                            device.
void RegRead (unsigned char reg_addr, unsigned char * reg_value,
              unsigned char device_addr); - Function to read from a register on
                                            the device where reg_addr is the
                                            register address, reg_value is the
                                            value the register is set to and
                                            device_addr is the address of the
                                            device.
