// pcint_pwm_joystick.c - Teensy 2.0++ (AT90USB1286) using PCINT2 group for 4-channel RC PWM to USB joystick

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include "Descriptors.h"
#include "Joystick.h"
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/USB/Class/Device/HIDClassDevice.h>

#define NUM_CHANNELS 4
#define PCINT_START 4  // PB4

const bool invert_axis[NUM_CHANNELS] = {
    true, // Channel 0 (X)
    false,  // Channel 1 (Y) -- inverted
    false, // Channel 2 (Z)
    false  // Channel 3 (if extended)
};

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevJoystickHIDReportBuffer[sizeof(USB_JoystickReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
*  passed to all HID Class driver functions, so that multiple instances of the same class
*  within a device can be differentiated from one another.
*/
USB_ClassInfo_HID_Device_t Joystick_HID_Interface =
{
    .Config =
    {
        .InterfaceNumber              = INTERFACE_ID_Joystick,
        .ReportINEndpoint             =
        {
            .Address              = JOYSTICK_EPADDR,
            .Size                 = JOYSTICK_EPSIZE,
            .Banks                = 1,
        },
        .PrevReportINBuffer           = PrevJoystickHIDReportBuffer,
        .PrevReportINBufferSize       = sizeof(PrevJoystickHIDReportBuffer),
    },
};

volatile uint16_t ch_pulse[NUM_CHANNELS] = {1500, 1500, 1500, 1500};
volatile uint16_t ch_start[NUM_CHANNELS] = {0};
volatile uint8_t  ch_edge[NUM_CHANNELS] = {0};

volatile uint8_t prev_pinb = 0;

ISR(PCINT0_vect) {
    uint8_t pinb = PINB;
    uint8_t changed = pinb ^ prev_pinb;
    uint16_t now = TCNT1;
    prev_pinb = pinb;

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        uint8_t mask = (1 << (PCINT_START + i));
        if (changed & mask) {
            if (pinb & mask) {
                ch_start[i] = now;
                ch_edge[i] = 1;
                } else if (ch_edge[i]) {
                ch_pulse[i] = now - ch_start[i];
                ch_edge[i] = 0;
            }
        }
    }
}

void setup_inputs(void) {
    // PB4-PB7 = inputs with pull-ups
    DDRB &= ~((1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));
    PORTB |= (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7);

    // Enable PCINT for PB4-PB7 (PCINT4-PCINT7)
    PCMSK0 |= (1 << PCINT4) | (1 << PCINT5) | (1 << PCINT6) | (1 << PCINT7);

    // Enable Pin Change Interrupt 0
    PCICR |= (1 << PCIE0);

    // Save current state of PORTB
    prev_pinb = PINB;

    // Timer1 @ 2 MHz (prescaler 8)
    TCCR1B = (1 << CS11); // clk/8
    TCNT1 = 0;
}

int main(void) {
    // Disable watchdog (in case it was enabled by bootloader)
    MCUSR &= ~(1 << WDRF);  // Clear watchdog reset flag
    wdt_disable();

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    DDRD |= (1 << PD6);
    setup_inputs();
    USB_Init();
    sei();

    while (1) {
        HID_Device_USBTask(&Joystick_HID_Interface);
        USB_USBTask();
    }
}
/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    ConfigSuccess &= HID_Device_ConfigureEndpoints(&Joystick_HID_Interface);

    USB_Device_EnableSOFEvents();
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
    HID_Device_ProcessControlRequest(&Joystick_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
    HID_Device_MillisecondElapsed(&Joystick_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
*
*  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
*  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
*  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
*  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
*  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
*
*  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
*/
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
    uint8_t* const ReportID,
    const uint8_t ReportType,
    void* ReportData,
    uint16_t* const ReportSize)
{
    PORTD ^= (1 << PD6); // LED toggle for debug

    USB_JoystickReport_Data_t* report = (USB_JoystickReport_Data_t*)ReportData;

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        int16_t centered = (int16_t)ch_pulse[i] - 3000;  // 1500 µs center
        // Limit to -1000…+1000 (i.e., ±500 µs)
        if (centered < -1000) centered = -1000;
        if (centered >  1000) centered =  1000;
        int8_t mapped = centered / 10;  // Map to approx -100…100
        if (invert_axis[i]) {
            mapped = -mapped;
        }
        switch (i) {
            case 0: report->X = mapped; break;
            case 1: report->Y = mapped; break;
            case 2: report->Z = mapped; break;
            // Extend here for additional channels if needed
        }
    }

    *ReportSize = sizeof(USB_JoystickReport_Data_t);
    return true;
}


/** HID class driver callback function for the processing of HID reports from the host.
*
*  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
*  \param[in] ReportID    Report ID of the received report from the host
*  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
*  \param[in] ReportData  Pointer to a buffer where the received report has been stored
*  \param[in] ReportSize  Size in bytes of the received HID report
*/
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
    const uint8_t ReportID,
    const uint8_t ReportType,
    const void* ReportData,
    const uint16_t ReportSize)
{
    // Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}
