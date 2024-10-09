#ifndef PWM_H_
#define PWM_H_

#define MOT_TOP_LEFT      32
#define MOT_TOP_RIGHT     23
#define MOT_BOTTOM_LEFT   27
#define MOT_BOTTOM_RIGHT  19
#define GREEN_LED         17
#define RED_LED           16

#define LEDC_FREQUENCY          (20000) // Frequency in Hertz. Set frequency at 20 kHz

void motors_init(void);
void motors_stop(void);
void configure_leds(void);
void motors_PWM(uint32_t id, uint16_t ithrust);

#endif