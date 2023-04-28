#include "PicoBattery.h"
#include <string.h>
#include "hardware/adc.h"

#define BAT_ADC_PIN     (29)
#define BAR_CHANNEL     (3)

static const float conversion_factor = 3.3f / (1 << 12) * 2;
float Read(Battery* bat)
{
    PicoBattery* this = (PicoBattery*)bat;
    this->read = adc_read() * conversion_factor;

    if (this->read < this->load)
    {
        if (this->read > this->max)
        {
            this->max = this->read;
        }

        if (this->read < this->min)
        {
            this->min = this->read;
            // if (plosa->is_sleeping && plosa->BRIGHTNESS == 0)
            // {
            //     dosave();
            // }
        }

        this->dif = this->max - this->min;
    }

    return this->read;
}

void PicoBattery_Init(PicoBattery* out)
{
    PicoBattery battery = 
    {
        .Base = 
        {
            .Read = Read
        },
    };

    // ADC
    adc_init();
    adc_gpio_init(BAT_ADC_PIN);
    adc_select_input(BAR_CHANNEL);

    memcpy(&out->Base, &battery.Base, sizeof(Battery));
}