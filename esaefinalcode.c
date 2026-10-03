/*
================================================================================
  PIC16F877A WATER QUALITY MONITORING & AUTOMATION SYSTEM
================================================================================
  Overview for Beginners:
  - This system monitors three main parameters:
      1. Turbidity (Water Clarity) using an Analog Sensor connected to AN0 (RA0).
      2. Dissolved Oxygen using a Digital I2C Sensor (Address 0x73).
      3. Water Level using a Digital Switch on RB0.
  - Based on these readings, it automatically controls three actuators on PORTD:
      1. Filter Pump (RD0)
      2. Aerator Pump (RD1)
      3. Refill Valve (RD2)
================================================================================
*/

// =============================================================================
// 1. CONFIGURATION BITS SETUP
// =============================================================================
// Configuration bits tell the microcontroller chip how to physically operate 
// before it even starts running your code.

#pragma config FOSC = HS        // Oscillator Selection: HS (High Speed) Crystal/Resonator (8 MHz)
#pragma config WDTE = OFF       // Watchdog Timer Enable: OFF (Prevents MCU from auto-resetting periodically)
#pragma config PWRTE = ON       // Power-up Timer Enable: ON (Adds a brief delay at power-up for voltage to stabilize)
#pragma config BOREN = ON       // Brown-out Reset Enable: ON (Resets chip if supply voltage drops dangerously low)
#pragma config LVP = OFF        // Low-Voltage Programming Enable: OFF (Frees up pin RB3 for normal I/O use)
#pragma config CPD = OFF        // Data EEPROM Code Protection: OFF (EEPROM memory is readable)
#pragma config WRT = OFF        // Flash Program Memory Write Protection: OFF (Program memory can be overwritten)
#pragma config CP = OFF         // Code Protection: OFF (Flash code can be read by external programmers)

// =============================================================================
// 2. LIBRARIES AND CLOCK DEFINITION
// =============================================================================

#include <xc.h>                 // Main XC8 compiler header (contains register and pin definitions)

#define _XTAL_FREQ 8000000      // Tells the compiler our crystal frequency is 8 MHz (8,000,000 Hz).
                                // Required for delay functions like __delay_ms() to calculate exact timing.

// =============================================================================
// 3. HARDWARE PIN DEFINITIONS (ALIASES FOR EASY READING)
// =============================================================================
// Creating descriptive nicknames for PORT registers makes code much easier to read and maintain.

// Define Output Pins (PORTD) - Actuators driven by output signals
#define FILTER_PUMP   RD0       // Pin RD0 (Pin 19 on PIC16F877A DIP40 package) -> Controls Water Filter Pump
#define AERATOR_PUMP  RD1       // Pin RD1 (Pin 20) -> Controls Aerator Pump to inject oxygen
#define REFILL_VALVE  RD2       // Pin RD2 (Pin 21) -> Controls Water Refill Valve

// Define Input Pin (PORTB) - Sensor input
#define LEVEL_SWITCH  RB0       // Pin RB0 (Pin 33) -> Digital level switch (0 = Low water, 1 = Full water)

// =============================================================================
// 4. SYSTEM CALIBRATION THRESHOLDS
// =============================================================================

#define TURBIDITY_THRESHOLD 512 // 10-bit ADC threshold (0 to 1023 range). 512 = ~2.5 Volts midpoint.
#define OXYGEN_THRESHOLD    87  // Calibrated for 15% threshold based on Proteus model scaling

// =============================================================================
// 5. I2C COMMUNICATION FUNCTIONS (COMMUNICATING WITH DIGITAL SENSORS)
// =============================================================================
/*
  I2C is a 2-wire serial protocol:
  - SCL (Serial Clock): Pin RC3 -> Synchronizes data transmission.
  - SDA (Serial Data):  Pin RC4 -> Sends and receives data bits.
*/

// Initialize PIC16F877A as an I2C Master device
void I2C_Master_Init(const unsigned long c) {
    SSPCON = 0b00101000;            // SSP Module Setup:
                                    // Bit 5 (SSPEN=1): Enables Master Synchronous Serial Port (MSSP)
                                    // Bits 3-0 (0100): Sets mode to I2C Master mode, clock = FOSC / (4 * (SSPADD+1))
    
    SSPCON2 = 0;                    // Clear all I2C control bits (Start, Stop, Ack bits cleared)
    
    // Calculate and set clock speed register (e.g., 100 kHz standard speed)
    SSPADD = (_XTAL_FREQ/(4*c))-1;  // Math formula for I2C baud rate generator
    
    SSPSTAT = 0;                    // Slew rate control enabled for standard speed (100kHz)
    
    TRISC3 = 1;                     // Set SCL (RC3) pin as Input (Required by PIC hardware for I2C)
    TRISC4 = 1;                     // Set SDA (RC4) pin as Input (Required by PIC hardware for I2C)
}

// Helper Function: Waits until the I2C bus becomes idle (not busy doing transmission)
void I2C_Master_Wait(void) {
    // SSPSTAT bit 2 (R_nW) checks if transfer is in progress.
    // SSPCON2 lower 5 bits check if Start, Repeated Start, Stop, Receive, or Ack are actively running.
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F)); // Loop and pause while busy
}

// Transmit a START signal on the I2C bus to inform devices a transfer is starting
void I2C_Master_Start(void) {
    I2C_Master_Wait();             // Ensure bus is free
    SEN = 1;                        // Set SEN bit (Start Enable) in SSPCON2 to send START condition
}

// Transmit a STOP signal on the I2C bus to end communication
void I2C_Master_Stop(void) {
    I2C_Master_Wait();             // Ensure bus is free
    PEN = 1;                        // Set PEN bit (Stop Enable) in SSPCON2 to send STOP condition
}

// Transmit 1 byte of data or an address over the I2C bus
void I2C_Master_Write(unsigned d) {
    I2C_Master_Wait();             // Ensure bus is free
    SSPBUF = d;                     // SSPBUF is the hardware buffer register; loading data into it initiates transmission
}

// Custom Function: Read Oxygen value from DFRobot SEN0322 Sensor module via I2C protocol
unsigned short I2C_Read_Sensor(void) {
    unsigned short rx_data = 255;   // Default fallback value (indicates error/no read)
    
    // Step 1: Tell sensor which register we want to read from
    I2C_Master_Start();             // Send START condition
    I2C_Master_Write(0xE6);         // Send Sensor Device Address in WRITE mode (7-bit address 0x73 shifted left + Write bit 0 = 0xE6)
    I2C_Master_Write(0x03);         // Send Register Address 0x03 (Target register containing Oxygen Integer value)
    I2C_Master_Stop();              // Send STOP condition to complete register selection request

    // Step 2: Request and read the data byte back from the sensor
    I2C_Master_Start();             // Send fresh START condition (Restart)
    I2C_Master_Write(0xE7);         // Send Sensor Device Address in READ mode (7-bit address 0x73 shifted left + Read bit 1 = 0xE7)
    
    I2C_Master_Wait();              // Wait for address byte transmission
    RCEN = 1;                       // Set RCEN bit (Receive Enable) to tell hardware to receive 1 byte from sensor
    I2C_Master_Wait();              // Wait for reception complete
    rx_data = SSPBUF;               // Copy incoming data byte out of hardware receiver buffer into variable
    
    I2C_Master_Wait();              // Wait for bus ready
    ACKDT = 1;                      // Prepare Acknowledge bit: 1 = NACK (Not Acknowledge)
    ACKEN = 1;                      // Send the NACK signal to sensor (signals we are finished reading)
    I2C_Master_Stop();              // Send STOP condition to free I2C bus
    
    return rx_data;                 // Return collected 8-bit sensor value
}

// =============================================================================
// 6. ANALOG-TO-DIGITAL CONVERTER (ADC) FUNCTIONS
// =============================================================================
/*
  The ADC converts an incoming continuous voltage (0V to 5V) into a digital 
  number from 0 to 1023 (10-bit resolution).
*/

// Initialize ADC Peripheral
void ADC_Init(void) {
    ADCON0 = 0x41; // binary 01000001:
                   // Bits 7-6: ADCS = 01 -> ADC conversion clock set to Fosc/8
                   // Bits 5-3: CHS  = 000 -> Default Channel selected is AN0
                   // Bit 2:    GO/DONE = 0 (Idle)
                   // Bit 0:    ADON = 1 -> Power ON the ADC module
                   
    ADCON1 = 0x80; // binary 10000000:
                   // Bit 7: ADFM = 1 -> Right Justified result format (10-bit value split across 2 bytes)
                   // Bits 3-0: PCFG = 0000 -> Configure AN0 through AN7 pins as Analog inputs
}

// Read raw 10-bit numerical value from a selected analog channel pin
unsigned int ADC_Read(unsigned char channel) {
    ADCON0 &= 0xC5;                 // Clear channel selection bits (bits 3, 4, 5) using bitwise AND mask
    ADCON0 |= (channel << 3);       // Shift input channel number into position and set it in ADCON0
    __delay_ms(2);                  // Acquisition Delay: Short pause for internal holding capacitor to sample input voltage
    GO_nDONE = 1;                   // Set GO/DONE bit to start ADC conversion process
    while(GO_nDONE);                // Poll bit and wait until hardware clears it (cleared automatically when conversion finishes)
    
    // Merge two 8-bit registers (ADRESH and ADRESL) into one 16-bit result:
    // ADRESH holds the high 2 bits (shifted left by 8 positions)
    // ADRESL holds the lower 8 bits
    return ((ADRESH << 8) + ADRESL); 
}

// =============================================================================
// 7. MAIN PROGRAM LOOP
// =============================================================================

void main(void) {
    // -------------------------------------------------------------------------
    // HARDWARE PORT DIRECTION INITIALIZATION (TRIS Registers)
    // Rule: TRIS bit = 1 -> INPUT, TRIS bit = 0 -> OUTPUT ("1" looks like "I", "0" looks like "O")
    // -------------------------------------------------------------------------
    TRISA = 0xFF;  // Set PORTA all pins as Inputs (RA0 receives Turbidity Analog signal)
    TRISB = 0xFF;  // Set PORTB all pins as Inputs (RB0 receives Water Level Switch signal)
    TRISD = 0x00;  // Set PORTD all pins as Outputs (Drives actuators: Pumps and Valves)
    
    PORTD = 0x00;  // Initialize all PORTD output pins to LOW (0V) -> All relays/actuators turned OFF at start
    
    // -------------------------------------------------------------------------
    // INITIALIZE PERIPHERALS
    // -------------------------------------------------------------------------
    ADC_Init();              // Turn ON and set up ADC hardware module
    I2C_Master_Init(100000); // Turn ON and set up I2C hardware at 100 kHz standard clock speed

    // -------------------------------------------------------------------------
    // INFINITE EXECUTION LOOP (Runs continuously while MCU is powered)
    // -------------------------------------------------------------------------
    while(1) {
        // --- STEP 1: Read Turbidity (Analog Sensor on Channel 0 / RA0) ---
        unsigned int turbidity_level = ADC_Read(0); // Value range: 0 to 1023

        // --- STEP 2: Read Dissolved Oxygen (I2C Digital Sensor) ---
        unsigned short oxygen_level = I2C_Read_Sensor(); // Raw byte read from sensor

        // --- STEP 3: Process Turbidity Logic (Filter Pump Control) ---
        // Higher ADC value = clear water; Lower ADC value = dirty/cloudy water
        if (turbidity_level < TURBIDITY_THRESHOLD) {
            FILTER_PUMP = 1; // Water is dirty/turbid -> Turn ON Filter Pump (RD0 = High)
        } else {
            FILTER_PUMP = 0; // Water clarity is good -> Turn OFF Filter Pump (RD0 = Low)
        }

        // --- STEP 4: Process Oxygen Logic (Aerator Pump Control) ---
        // Turns ON when Proteus slider is 15 or lower, OFF at 16 or above
        if (oxygen_level < OXYGEN_THRESHOLD) {
            AERATOR_PUMP = 1; // Dissolved oxygen level is low -> Turn ON Aerator Pump (RD1 = High)
        } else {
            AERATOR_PUMP = 0; // Oxygen level is sufficient -> Turn OFF Aerator Pump (RD1 = Low)
        }

        // --- STEP 5: Process Water Level Logic (Refill Valve Control) ---
        // Digital Switch: 0 = Low Water level detected, 1 = Tank full
        if (LEVEL_SWITCH == 0) {
            REFILL_VALVE = 1; // Water level is low -> Open Refill Valve (RD2 = High)
        } else {
            REFILL_VALVE = 0; // Tank is full -> Close Refill Valve (RD2 = Low)
        }

        // --- STEP 6: Delay between iterations ---
        __delay_ms(200); // Wait 200 milliseconds before reading sensors again (prevents rapid switching)
    }
}