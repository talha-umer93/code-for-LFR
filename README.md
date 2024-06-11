#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Motor control pins
#define MOTOR1_PIN PB0
#define MOTOR2_PIN PB1

// Ultrasonic sensor pins
#define TRIGGER_PIN PD6
#define ECHO_PIN PD7

// IR sensor pins
#define LEFT_IR_PIN PC0
#define RIGHT_IR_PIN PC1

// Function prototypes
void init_ADC();
uint16_t read_ADC(uint8_t channel);
void init_PWM();
void set_motor_speed(uint8_t motor, uint8_t speed, uint8_t direction);
uint16_t read_ultrasonic();
void init_ultrasonic();
void init_color_sensor();
void read_color(uint16_t *r, uint16_t *g, uint16_t *b);

int main(void) {
    // Initialize peripherals
    DDRB |= (1 << MOTOR1_PIN) | (1 << MOTOR2_PIN);
    init_ADC();
    init_PWM();
    init_ultrasonic();
    init_color_sensor();

    while (1) {
        // Read IR sensors
        uint16_t leftIRValue = read_ADC(LEFT_IR_PIN);
        uint16_t rightIRValue = read_ADC(RIGHT_IR_PIN);

        // Line following logic
        if (leftIRValue < 512 && rightIRValue < 512) {
            // Move forward
            set_motor_speed(1, 200, 1);
            set_motor_speed(2, 200, 1);
        } else if (leftIRValue > 512 && rightIRValue < 512) {
            // Turn right
            set_motor_speed(1, 200, 1);
            set_motor_speed(2, 200, 0);
        } else if (leftIRValue < 512 && rightIRValue > 512) {
            // Turn left
            set_motor_speed(1, 200, 0);
            set_motor_speed(2, 200, 1);
        } else {
            // Stop
            set_motor_speed(1, 0, 1);
            set_motor_speed(2, 0, 1);
        }

        // Obstacle detection logic
        uint16_t distance = read_ultrasonic();
        if (distance > 0 && distance < 20) {
            // Obstacle detected, stop the motors
            set_motor_speed(1, 0, 1);
            set_motor_speed(2, 0, 1);
        }

        // Color detection logic
        uint16_t r, g, b;
        read_color(&r, &g, &b);
        // Print color values (use UART or other method to debug)
        
        _delay_ms(100);
    }
}

void init_ADC() {
    ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler of 64
}

uint16_t read_ADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select ADC channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC;
}

void init_PWM() {
    // Initialize Timer0 for PWM
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
    TCCR0B = (1 << CS01); // Prescaler of 8
    DDRD |= (1 << PD5) | (1 << PD6); // PD5 and PD6 as output for PWM
}

void set_motor_speed(uint8_t motor, uint8_t speed, uint8_t direction) {
    if (motor == 1) {
        if (direction == 1) {
            PORTB |= (1 << MOTOR1_PIN);
        } else {
            PORTB &= ~(1 << MOTOR1_PIN);
        }
        OCR0A = speed; // Set speed
    } else if (motor == 2) {
        if (direction == 1) {
            PORTB |= (1 << MOTOR2_PIN);
        } else {
            PORTB &= ~(1 << MOTOR2_PIN);
        }
        OCR0B = speed; // Set speed
    }
}

void init_ultrasonic() {
    // Initialize ultrasonic sensor pins
    DDRD |= (1 << TRIGGER_PIN);
    DDRD &= ~(1 << ECHO_PIN);
}

uint16_t read_ultrasonic() {
    // Send trigger pulse
    PORTD &= ~(1 << TRIGGER_PIN);
    _delay_us(2);
    PORTD |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIGGER_PIN);

    // Measure echo pulse width
    while (!(PIND & (1 << ECHO_PIN)));
    uint32_t count = 0;
    while (PIND & (1 << ECHO_PIN)) {
        _delay_us(1);
        count++;
    }
    return count / 58; // Convert to cm
}

void init_color_sensor() {
    // Initialize I2C and color sensor
    // Specific implementation depends on the color sensor used
}

void read_color(uint16_t *r, uint16_t *g, uint16_t *b) {
    // Read color values from the sensor
    // Specific implementation depends on the color sensor used
}
