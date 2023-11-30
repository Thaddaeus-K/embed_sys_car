#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define BUTTON_A_PIN 21
#define BUTTON_B_PIN 20
#define UART_TX_PIN 0
#define UART_RX_PIN 1


int main() {
    uart_init(UART_ID, BAUD_RATE);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    stdio_init_all();
    // stdio_usb_init();
    printf("ADC Example, measuring GPIO26\n");

    adc_init();
    adc_gpio_init(26);  // Replace 26 with the pin connected to the X-axis potentiometer
    adc_gpio_init(27);  // Replace 27 with the pin connected to the Y-axis potentiometer
    
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_set_pulls(BUTTON_A_PIN, true, false);

    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_set_pulls(BUTTON_B_PIN, true, false);

    uint16_t x_value;
    uint16_t y_value;


    //char* joystickstate = "";  // Use a character pointer

    int joystickstate;
    int abuttonstate;
    int bbuttonstate;
    int buttontriggerstate;
    int buttontriggerstate2;
    while (1) {
        // adc_select_input(0); // Select ADC channel 0 for X-axis
        // x_value = adc_read();

        // adc_select_input(1); // Select ADC channel 1 for Y-axis
        // y_value = adc_read();
        
        // joystickstate = 100;
        adc_select_input(0); // Select ADC channel 0 for X-axis
        x_value = adc_read();

        adc_select_input(1); // Select ADC channel 1 for Y-axis
        y_value = adc_read();

        // Read the state of Button A
        abuttonstate = gpio_get(BUTTON_A_PIN);
        bbuttonstate = gpio_get(BUTTON_B_PIN);

        if (abuttonstate==0)
        {
        
            if(buttontriggerstate == 1)
            {
                buttontriggerstate = 0;
                uart_putc_raw(UART_ID,'s');
                
            }
            else{
                buttontriggerstate=1;
                uart_putc_raw(UART_ID,'l');
            }
        
        }
        if (bbuttonstate==0)
        {
                if(buttontriggerstate2 == 1)
                {
                    buttontriggerstate2 = 0;
                    uart_putc_raw(UART_ID,'s');
                    
                }
            else
                {
                    buttontriggerstate2=1;
                    uart_putc_raw(UART_ID,'r');
                }
        }
        char carstate;

        //printf("X-axis: %u, Y-axis: %u\n", x_value, y_value);

        // Check the range of X-axis value
        if ((x_value >= 2000 && x_value < 4000) && (y_value >= 2000 && y_value < 4000) ) {
            //joystickstate = "Still";
            joystickstate = 0;
            carstate = 'a';
            // uart_putc_raw(UART_ID, '0');
        } else if((x_value >= 2000 && x_value < 4000) && y_value >= 4000)  {
            //joystickstate = "Top";
            joystickstate = 1;
            carstate = 'b';
            // uart_putc_raw(UART_ID, '1');
        } else if((x_value >= 2000 && x_value < 4000) && y_value < 1000)  {
            //joystickstate = "Bottom";
            joystickstate = 2;
            carstate = 'z';
            // uart_putc_raw(UART_ID, '2');
        }else if(x_value <1000 && (y_value >= 2000 && y_value < 4000))  {
            //joystickstate = "Left";
            joystickstate = 3;
            carstate = 'd';
            // uart_putc_raw(UART_ID, '3');
        } else if (x_value >= 4000 && (y_value >= 2000 && y_value < 4000)) {
            //joystickstate = "Right";
            joystickstate = 4;
            carstate = 'e';
            // uart_putc_raw(UART_ID, '4');
        } else if (x_value < 1000 && y_value >= 4000) {
            //joystickstate = "Top Left";
            joystickstate = 5;
            carstate = 'f';
            // uart_putc_raw(UART_ID, '5');
        } else if (x_value >= 4000 && y_value >= 4000) {
            //joystickstate = "Top Right";
            joystickstate = 6;
            carstate = 'g';
            // uart_putc_raw(UART_ID, '6');
        } else if (x_value < 1000 && y_value < 1000) {
            //joystickstate = "Bottom Left";
            joystickstate = 7;
            carstate = 'h';
            // uart_putc_raw(UART_ID, '7');
        } else if (x_value >= 4000 && y_value < 1000) {
            //joystickstate = "Bottom Right";
            joystickstate = 8;
            carstate = 'j';
            
        }


        uart_putc_raw(UART_ID,carstate);
        // uart_putc_raw(UART_ID, "1");

        printf("Joystick State: %d\n", joystickstate);
        printf("Button A State: %d\n", abuttonstate);
        printf("Button B State: %d\n", bbuttonstate);

        sleep_ms(100);  // Read values every 100 milliseconds


    }

    return 0;
}
