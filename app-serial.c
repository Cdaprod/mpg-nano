/*
 * MPG-Nano - Firmware and UCCNC plugin for Arduino Nano based serial-over-USB
 * interface for modified 4-axis Chinese MPG pendant.
 *
 * https://github.com/mattbucknall/mpg-nano
 *
 * Changes to include additional selectors for 5 & 6
 *
 
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <stdbool.h>

#include "app-encoder.h"
#include "app-serial.h"
#include "app-switch.h"

#define APP_SERIAL_TX_BUFFER_SIZE       16
#define APP_SERIAL_UCSRXB_RECEIVE       ((1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0))
#define APP_SERIAL_UCSRXB_TRANSMIT      (APP_SERIAL_UCSRXB_RECEIVE | (1 << UDRIE0))

typedef enum {
    APP_SERIAL_STATE_READY,
    APP_SERIAL_STATE_HAVE_RESET_REQ,
    APP_SERIAL_STATE_HAVE_STATUS_REQ,
    APP_SERIAL_STATE_RESPONDING
} app_serial_state_t;

static volatile int8_t m_state;
static volatile char m_tx_buffer[APP_SERIAL_TX_BUFFER_SIZE];
static volatile uint8_t m_tx_index;
static volatile uint8_t m_tx_count;

ISR(USART_RX_vect) {
    uint8_t flags;
    char cmd;

    flags = UCSR0A;
    cmd = (char) UDR0;

    if (m_state != APP_SERIAL_STATE_READY || flags & (1 << FE0)) {
        return;
    }

    switch (cmd) {
    case 'R':
        m_state = APP_SERIAL_STATE_HAVE_RESET_REQ;
        break;

    case 'S':
        m_state = APP_SERIAL_STATE_HAVE_STATUS_REQ;
        break;

    default:
        break;
    }
}

ISR(USART_UDRE_vect) {
    if (m_tx_index < m_tx_count) {
        UDR0 = m_tx_buffer[m_tx_index++];
    } else {
        UCSR0B = APP_SERIAL_UCSRXB_RECEIVE;
        m_state = APP_SERIAL_STATE_READY;
    }
}

static void send_response(uint8_t n_chars) {
    m_state = APP_SERIAL_STATE_RESPONDING;
    m_tx_index = 0;
    m_tx_count = n_chars;
    UCSR0B = APP_SERIAL_UCSRXB_TRANSMIT;
}

static void nibble_to_hex(volatile char *buffer, uint8_t nibble) {
    buffer[0] = (char) ((nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10));
}

static void uint8_to_hex(volatile char *buffer, uint8_t value) {
    nibble_to_hex(buffer, value >> 4);
    nibble_to_hex(buffer + 1, value & 0xF);
}

static void uint16_to_hex(volatile char *buffer, uint16_t value) {
    uint8_to_hex(buffer, value >> 8);
    uint8_to_hex(buffer + 2, value & 0xFF);
}

void app_serial_loop(void) {
    uint16_t enc_delta;
    uint8_t switch_bits;

    switch (m_state) {
    case APP_SERIAL_STATE_HAVE_RESET_REQ:
        app_encoder_reset();

        m_tx_buffer[0] = '[';
        m_tx_buffer[1] = 'R';
        m_tx_buffer[2] = ']';
        m_tx_buffer[3] = '\r';
        m_tx_buffer[4] = '\n';

        send_response(5);
        break;

    case APP_SERIAL_STATE_HAVE_STATUS_REQ:
        enc_delta = (uint16_t) app_encoder_delta();

        switch_bits = (uint8_t) app_switch_axis();
        switch_bits |= ((uint8_t) app_switch_step()) << 3;
        switch_bits |= app_switch_e_stop() ? (1 << 6) : 0;

        m_tx_buffer[0] = '[';
        m_tx_buffer[1] = 'S';

        uint16_to_hex(&m_tx_buffer[2], enc_delta);
        uint8_to_hex(&m_tx_buffer[6], switch_bits);

        m_tx_buffer[8] = ']';
        m_tx_buffer[9] = '\r';
        m_tx_buffer[10] = '\n';

        send_response(11);
        break;

    default:
        break;
    }
}

void app_serial_init(void) {
    power_usart0_enable();

    UBRR0 = 25;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0B = APP_SERIAL_UCSRXB_RECEIVE;
}