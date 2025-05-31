// pcint_pwm_joystick.c - Teensy 2.0++ (AT90USB1286) using PCINT2 group for 4-channel RC PWM to USB joystick

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include "Descriptors.h"
#include "Joystick.h"
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/USB/Class/Device/HIDClassDevice.h>

#define NUM_CHANNELS 8
#define PCINT_START 0 // PB0

#define TIMER_CLOCK_MHZ 2

#define PULSE_MIN (1000 * TIMER_CLOCK_MHZ)   // 2000
#define PULSE_MAX (2000 * TIMER_CLOCK_MHZ)   // 4000
#define PULSE_CENTER (1500 * TIMER_CLOCK_MHZ)   // 3000
#define RANGE (PULSE_MAX - PULSE_MIN) // 2000

const bool invert_axis[NUM_CHANNELS] = {
    true,
    true,
    true,
    true,
    true,
    true,
    true,
    true
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

volatile uint16_t ch_pulse[NUM_CHANNELS] = {PULSE_CENTER, PULSE_CENTER, PULSE_CENTER, PULSE_CENTER, PULSE_CENTER, PULSE_CENTER, PULSE_CENTER, PULSE_CENTER};
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
    // PB0-PB7 = inputs with pull-ups
    DDRB &= ~0xFF;
    PORTB |= 0xFF;

    // Enable PCINT for PB0-PB7 (PCINT0-PCINT7)
    PCMSK0 |= 0xFF;

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
    PORTD ^= (1 << PD6);

    USB_JoystickReport_Data_t* report = (USB_JoystickReport_Data_t*)ReportData;

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        uint16_t pulse = ch_pulse[i];

        if (pulse < PULSE_MIN) pulse = PULSE_MIN;
        if (pulse > PULSE_MAX) pulse = PULSE_MAX;

        int16_t centered = (int16_t)pulse - PULSE_CENTER;

        if (invert_axis[i])
            centered = -centered;

        report->axes[i] = centered;
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
