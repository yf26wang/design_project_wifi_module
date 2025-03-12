#include <stdio.h>
#include <driver\gpio.h>
#include <freertos\FreeRTOS.h>
#include <freertos\task.h>

// Define the GPIO pin for the interrupt
#define PUSH_BUTTON1 GPIO_NUM_10

// LED Pin
#define LED_G GPIO_NUM_5

volatile BaseType_t button_flag, led_flag;

void IRAM_ATTR buttonInterrupt()
{
    button_flag = 1;
}

void app_main(void)
{
    button_flag = 0;

    // LED Pin
    gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);

    //EXT Interrupt
    gpio_set_direction(PUSH_BUTTON1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PUSH_BUTTON1, GPIO_PULLUP_ENABLE);


    gpio_set_intr_type(PUSH_BUTTON1, GPIO_INTR_LOW_LEVEL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PUSH_BUTTON1, buttonInterrupt, NULL);

    for(;;)
    {
        if(button_flag)
        {
            led_flag ^= 1;

            {
                vTaskDelay(300/portTICK_PERIOD_MS); // debounce
                gpio_set_level(LED_G, led_flag); //gpio_set_level(PUSH_BUTTON1, !gpio_get_level(PUSH_BUTTON1)));
                button_flag = 0;
            }

        }

        vTaskDelay(100/portTICK_PERIOD_MS);
    }

}