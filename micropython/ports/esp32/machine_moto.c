#include "py/runtime.h"
#include "py/mphal.h"
#include <math.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "modmachine.h"

#define PWM_DBG(...)
// #define PWM_DBG(...) mp_printf(&mp_plat_print, __VA_ARGS__)

// Total number of channels
#define MOTO_CHANNEL_MAX (LEDC_SPEED_MODE_MAX * LEDC_CHANNEL_MAX)
typedef struct _moto_chan_t {
    // Which channel has which GPIO pin assigned?
    // (-1 if not assigned)
    gpio_num_t pin;
    // Which channel has which timer assigned?
    // (-1 if not assigned)
    int timer_idx;
} moto_chan_t;
// List of PWM channels
STATIC moto_chan_t motochans[MOTO_CHANNEL_MAX];

// channel_idx is an index (end-to-end sequential numbering) for all channels
// available on the chip and described in motochans[]
#define MOTO_CHANNEL_IDX(mode, channel) (mode * LEDC_CHANNEL_MAX + channel)
#define MOTO_CHANNEL_IDX_TO_MODE(channel_idx) (channel_idx / LEDC_CHANNEL_MAX)
#define MOTO_CHANNEL_IDX_TO_CHANNEL(channel_idx) (channel_idx % LEDC_CHANNEL_MAX)

// Total number of  mototimers
#define MOTO_TIMER_MAX (LEDC_SPEED_MODE_MAX * LEDC_TIMER_MAX)
// List of timer configs
STATIC ledc_timer_config_t  mototimers[MOTO_TIMER_MAX];

// timer_idx is an index (end-to-end sequential numbering) for all  mototimers
// available on the chip and configured in  mototimers[]
#define MOTO_TIMER_IDX(mode, timer) (mode * LEDC_TIMER_MAX + timer)
#define MOTO_TIMER_IDX_TO_MODE(timer_idx) (timer_idx / LEDC_TIMER_MAX)
#define MOTO_TIMER_IDX_TO_TIMER(timer_idx) (timer_idx % LEDC_TIMER_MAX)

// Params for PW operation
// 5khz is default frequency
#define MOTO_PWFREQ (5000)

// 10-bit resolution (compatible with esp8266 PWM)
#define MOTO_PWRES (LEDC_TIMER_10_BIT)

// Config of timer upon which we run all PWM'ed GPIO pins
STATIC bool moto_inited = false;

// MicroPython PWM object struct
typedef struct _machine_moto_pwm_obj_t {
    mp_obj_base_t base;
    gpio_num_t pin;
    bool active;
    int mode;
    int channel;
    int timer;
} machine_moto_pwm_obj_t;

typedef struct _machine_moto_obj_t {
    mp_obj_base_t base;
    machine_moto_pwm_obj_t pina;
    machine_moto_pwm_obj_t pinb;
    int speed;
} machine_moto_obj_t;

STATIC void moto_init(void) {
    // Initial condition: no channels assigned
    for (int i = 0; i < MOTO_CHANNEL_MAX; ++i) {
        motochans[i].pin = -1;
        motochans[i].timer_idx = -1;
    }

    // Prepare all  mototimers config
    // Initial condition: no  mototimers assigned
    for (int i = 0; i < MOTO_TIMER_MAX; ++i) {
         mototimers[i].duty_resolution = MOTO_PWRES;
        // unset timer is -1
         mototimers[i].freq_hz = -1;
         mototimers[i].speed_mode = MOTO_TIMER_IDX_TO_MODE(i);
         mototimers[i].timer_num = MOTO_TIMER_IDX_TO_TIMER(i);
         mototimers[i].clk_cfg = LEDC_AUTO_CLK;
    }
}

STATIC void configure_channel(machine_moto_pwm_obj_t *self) {
    ledc_channel_config_t cfg = {
        .channel = self->channel,
        .duty = (1 << ( mototimers[MOTO_TIMER_IDX(self->mode, self->timer)].duty_resolution)) / 2,
        .gpio_num = self->pin,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = self->mode,
        .timer_sel = self->timer,
    };
    if (ledc_channel_config(&cfg) != ESP_OK) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("PWM not supported on Pin(%d)"), self->pin);
    }
}

STATIC void set_freq(int newval, ledc_timer_config_t *timer) {
    // If already set, do nothing
    if (newval == timer->freq_hz) {
        return;
    }

    // Find the highest bit resolution for the requested frequency
    if (newval <= 0) {
        newval = 1;
    }
    unsigned int res = 0;
    for (unsigned int i = LEDC_APB_CLK_HZ / newval; i > 1; i >>= 1) {
        ++res;
    }
    if (res == 0) {
        res = 1;
    } else if (res > MOTO_PWRES) {
        // Limit resolution to PWRES to match units of our duty
        res = MOTO_PWRES;
    }

    // Configure the new resolution and frequency
    timer->duty_resolution = res;
    timer->freq_hz = newval;

    // set freq
    esp_err_t err = ledc_timer_config(timer);
    if (err != ESP_OK) {
        if (err == ESP_FAIL) {
            PWM_DBG("timer timer->speed_mode %d, timer->timer_num %d, timer->clk_cfg %d, timer->freq_hz  %d, timer->duty_resolution %d)", timer->speed_mode, timer->timer_num, timer->clk_cfg, timer->freq_hz, timer->duty_resolution);
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("bad frequency %d"), newval);
        } else {
            check_esp_err(err);
        }
    }
}


STATIC void set_duty(machine_moto_pwm_obj_t *self, int duty) {
    if ((duty < 0) || (duty > (1 << MOTO_PWRES) - 1)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("duty must be between 0 and %u"), (1 << MOTO_PWRES) - 1);
    }
    duty &= (1 << MOTO_PWRES) - 1;
    duty >>= MOTO_PWRES -  mototimers[MOTO_TIMER_IDX(self->mode, self->timer)].duty_resolution;
    check_esp_err(ledc_set_duty(self->mode, self->channel, duty));
    check_esp_err(ledc_update_duty(self->mode, self->channel));
    // check_esp_err(ledc_set_duty_and_update(self->mode, self->channel, duty, (1 << PWRES) - 1)); // thread safe function ???

    // Bug: Sometimes duty is not set right now.
    // See https://github.com/espressif/esp-idf/issues/7288
    /*
    if (duty != get_duty(self)) {
        PWM_DBG("\n duty_set %u %u %d %d \n", duty, get_duty(self), PWRES,  mototimers[TIMER_IDX(self->mode, self->timer)].duty_resolution);
    }
    */
}

STATIC int get_duty(machine_moto_pwm_obj_t *self) {
    uint32_t duty = ledc_get_duty(self->mode, self->channel);
    duty <<= MOTO_PWRES - mototimers[MOTO_TIMER_IDX(self->mode, self->timer)].duty_resolution;
    return duty;
}

/******************************************************************************/
#define SAME_FREQ_ONLY (true)
#define SAME_FREQ_OR_FREE (false)
#define ANY_MODE (-1)
// Return timer_idx. Use MOTO_TIMER_IDX_TO_MODE(timer_idx) and MOTO_TIMER_IDX_TO_TIMER(timer_idx) to get mode and timer
STATIC int find_timer(int freq, bool same_freq_only, int mode) {
    int free_timer_idx_found = -1;
    // Find a free PWM Timer using the same freq
    for (int timer_idx = 0; timer_idx < MOTO_TIMER_MAX; ++timer_idx) {
        if ((mode == ANY_MODE) || (mode == MOTO_TIMER_IDX_TO_MODE(timer_idx))) {
            if ( mototimers[timer_idx].freq_hz == freq) {
                // A timer already uses the same freq. Use it now.
                return timer_idx;
            }
            if (!same_freq_only && (free_timer_idx_found == -1) && ( mototimers[timer_idx].freq_hz == -1)) {
                free_timer_idx_found = timer_idx;
                // Continue to check if a channel with the same freq is in use.
            }
        }
    }

    return free_timer_idx_found;
}

// Return true if the timer is in use in addition to current channel
STATIC bool is_timer_in_use(int current_channel_idx, int timer_idx) {
    for (int i = 0; i < MOTO_CHANNEL_MAX; ++i) {
        if ((i != current_channel_idx) && (motochans[i].timer_idx == timer_idx)) {
            return true;
        }
    }

    return false;
}

// Find a free PWM channel, also spot if our pin is already mentioned.
// Return channel_idx. Use CHANNEL_IDX_TO_MODE(channel_idx) and CHANNEL_IDX_TO_CHANNEL(channel_idx) to get mode and channel
STATIC int find_channel(int pin, int mode) {
    int avail_idx = -1;
    int channel_idx;
    for (channel_idx = 0; channel_idx < MOTO_CHANNEL_MAX; ++channel_idx) {
        if ((mode == ANY_MODE) || (mode == MOTO_CHANNEL_IDX_TO_MODE(channel_idx))) {
            if (motochans[channel_idx].pin == pin) {
                break;
            }
            if ((avail_idx == -1) && (motochans[channel_idx].pin == -1)) {
                avail_idx = channel_idx;
            }
        }
    }
    if (channel_idx >= MOTO_CHANNEL_MAX) {
        channel_idx = avail_idx;
    }
    return channel_idx;
}

/******************************************************************************/
// MicroPython bindings for PWM

STATIC void mp_machine_moto_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_moto_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "PWM(pin=%u", self->pin);
    if (self->active) {
        int duty = get_duty(self);
        mp_printf(print, ", freq=%u, duty=%u", ledc_get_freq(self->mode, self->timer), duty);
        mp_printf(print, ", resolution=%u",  mototimers[MOTO_TIMER_IDX(self->mode, self->timer)].duty_resolution);
        mp_printf(print, ", mode=%d, channel=%d, timer=%d", self->mode, self->channel, self->timer);
    }
    mp_printf(print, ")");
}

STATIC void mp_machine_moto_duty(machine_moto_pwm_obj_t *self, int speed) {
 
    int channel_idx = find_channel(self->pin, ANY_MODE);
    if (channel_idx == -1) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("out of PWM channels:%d"), MOTO_CHANNEL_MAX); // in all modes
    }

    int freq = MOTO_PWFREQ;

    int timer_idx = find_timer(freq, SAME_FREQ_OR_FREE, MOTO_CHANNEL_IDX_TO_MODE(channel_idx));
    if (timer_idx == -1) {
        timer_idx = find_timer(freq, SAME_FREQ_OR_FREE, ANY_MODE);
    }
    if (timer_idx == -1) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("out of PWM  mototimers:%d"), MOTO_TIMER_MAX); // in all modes
    }

    int mode = MOTO_TIMER_IDX_TO_MODE(timer_idx);
    if (MOTO_CHANNEL_IDX_TO_MODE(channel_idx) != mode) {
        // unregister old channel
        motochans[channel_idx].pin = -1;
        motochans[channel_idx].timer_idx = -1;
        // find new channel
        channel_idx = find_channel(self->pin, mode);
        if (MOTO_CHANNEL_IDX_TO_MODE(channel_idx) != mode) {
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("out of PWM channels:%d"), MOTO_CHANNEL_MAX); // in current mode
        }
    }
    self->mode = mode;
    self->timer = MOTO_TIMER_IDX_TO_TIMER(timer_idx);
    self->channel = MOTO_CHANNEL_IDX_TO_CHANNEL(channel_idx);

    // New PWM assignment
    if ((motochans[channel_idx].pin == -1) || (motochans[channel_idx].timer_idx != timer_idx)) {
        configure_channel(self);
        motochans[channel_idx].pin = self->pin;
    }
    motochans[channel_idx].timer_idx = timer_idx;
    self->active = true;

    // Set timer frequency
    set_freq(freq, & mototimers[timer_idx]);
    // Set duty cycle?

    int duty = (int)((float)(1023 / 100) * abs(speed));
    if (duty != -1) {
        set_duty(self, duty);
    }
    // Reset the timer if low speed
    if (self->mode == LEDC_LOW_SPEED_MODE) {
        check_esp_err(ledc_timer_rst(self->mode, self->timer));
    }
}

STATIC void mp_machine_moto_deinit(machine_moto_pwm_obj_t *self) {
    int chan = MOTO_CHANNEL_IDX(self->mode, self->channel);

    // Valid channel?
    if ((chan >= 0) && (chan < MOTO_CHANNEL_MAX)) {
        // Clean up timer if necessary
        if (!is_timer_in_use(chan, motochans[chan].timer_idx)) {
            check_esp_err(ledc_timer_rst(self->mode, self->timer));
            // Flag it unused
             mototimers[motochans[chan].timer_idx].freq_hz = -1;
        }

        // Mark it unused, and tell the hardware to stop routing
        check_esp_err(ledc_stop(self->mode, chan, 0));
        // Disable ledc signal for the pin
        // gpio_matrix_out(self->pin, SIG_GPIO_OUT_IDX, false, false);
        if (self->mode == LEDC_LOW_SPEED_MODE) {
            gpio_matrix_out(self->pin, LEDC_LS_SIG_OUT0_IDX + self->channel, false, true);
        } else {
            #if LEDC_SPEED_MODE_MAX > 1
            #if CONFIG_IDF_TARGET_ESP32
            gpio_matrix_out(self->pin, LEDC_HS_SIG_OUT0_IDX + self->channel, false, true);
            #else
            #error Add supported CONFIG_IDF_TARGET_ESP32_xxx
            #endif
            #endif
        }
        motochans[chan].pin = -1;
        motochans[chan].timer_idx = -1;
        self->active = false;
        self->mode = -1;
        self->channel = -1;
        self->timer = -1;
    }
}

STATIC mp_obj_t mp_machine_moto_make_new(const mp_obj_type_t *type,
    size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 2, MP_OBJ_FUN_ARGS_MAX, true);
    gpio_num_t pin_ida = machine_pin_get_id(args[0]);
    gpio_num_t pin_idb = machine_pin_get_id(args[1]);

    // create PWM object from the given pin
    machine_moto_obj_t *self = m_new_obj(machine_moto_obj_t);
    self->base.type = &machine_moto_type;
    self->pina.pin = pin_ida;
    self->pinb.pin = pin_idb;

    self->pina.active = false;
    self->pina.mode = -1;
    self->pina.channel = -1;
    self->pina.timer = -1;

    self->pinb.active = false;
    self->pinb.mode = -1;
    self->pinb.channel = -1;
    self->pinb.timer = -1;

    machine_moto_pwm_obj_t *self_pina = &(self->pina);
    machine_moto_pwm_obj_t *self_pinb = &(self->pinb);
    // start the PWM subsystem if it's not already running
    if (!moto_inited) {
        moto_init();
        moto_inited = true;
    }

    // start the PWM running for this channel
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    if (n_args == 3) {
        self->speed = mp_obj_get_int(args[2]);
        // printf("speed: %d \n",(self->speed));
        if (self->speed >= 0 && self->speed <= 100) {
            mp_machine_moto_duty((machine_moto_pwm_obj_t *)(self_pina), self->speed);
            mp_machine_moto_duty((machine_moto_pwm_obj_t *)(self_pinb), self->speed);
            mp_machine_moto_deinit((machine_moto_pwm_obj_t *)(self_pina));
        }else if (self->speed < 0 && self->speed >= -100) {
            mp_machine_moto_duty((machine_moto_pwm_obj_t *)(self_pina), self->speed);
            mp_machine_moto_duty((machine_moto_pwm_obj_t *)(self_pinb), self->speed);
            mp_machine_moto_deinit((machine_moto_pwm_obj_t *)(self_pinb));
        }else {
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("speed is -100 ~ 100"));
        }
    }

    return MP_OBJ_FROM_PTR(self);
}

STATIC void mp_machine_pwm_freq_set(machine_moto_pwm_obj_t *self, mp_int_t freq) {
    if (freq ==  mototimers[MOTO_TIMER_IDX(self->mode, self->timer)].freq_hz) {
        return;
    }

    int current_timer_idx = motochans[MOTO_CHANNEL_IDX(self->mode, self->channel)].timer_idx;
    bool current_in_use = is_timer_in_use(MOTO_CHANNEL_IDX(self->mode, self->channel), current_timer_idx);

    // Check if an already running timer with the same freq is running
    int new_timer_idx = find_timer(freq, SAME_FREQ_ONLY, self->mode);

    // If no existing timer was found, and the current one is in use, then find a new one
    if ((new_timer_idx == -1) && current_in_use) {
        // Have to find a new timer
        new_timer_idx = find_timer(freq, SAME_FREQ_OR_FREE, self->mode);

        if (new_timer_idx == -1) {
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("out of PWM  mototimers:%d"), MOTO_TIMER_MAX); // in current mode
        }
    }

    if ((new_timer_idx != -1) && (new_timer_idx != current_timer_idx)) {
        // Bind the channel to the new timer
        motochans[self->channel].timer_idx = new_timer_idx;

        if (ledc_bind_channel_timer(self->mode, self->channel, MOTO_TIMER_IDX_TO_TIMER(new_timer_idx)) != ESP_OK) {
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("failed to bind timer to channel"));
        }

        if (!current_in_use) {
            // Free the old timer
            check_esp_err(ledc_timer_rst(self->mode, self->timer));
            // Flag it unused
             mototimers[current_timer_idx].freq_hz = -1;
        }

        current_timer_idx = new_timer_idx;
    }
    self->mode = MOTO_TIMER_IDX_TO_MODE(current_timer_idx);
    self->timer = MOTO_TIMER_IDX_TO_TIMER(current_timer_idx);

    // Set the freq
    set_freq(freq, & mototimers[current_timer_idx]);

    // Reset the timer if low speed
    if (self->mode == LEDC_LOW_SPEED_MODE) {
        check_esp_err(ledc_timer_rst(self->mode, self->timer));
    }
}

STATIC void mp_machine_pwm_duty_set(machine_moto_pwm_obj_t *self, mp_int_t duty) {
    set_duty(self, duty);
}

STATIC mp_obj_t machine_moto_stop(mp_obj_t self_in) {
    machine_moto_obj_t *self = MP_OBJ_TO_PTR(self_in);
    machine_moto_pwm_obj_t *self_pina = &(self->pina);
    machine_moto_pwm_obj_t *self_pinb = &(self->pinb);
    mp_machine_moto_deinit((machine_moto_pwm_obj_t *)(self_pina));
    mp_machine_moto_deinit((machine_moto_pwm_obj_t *)(self_pinb));
 
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_moto_stop_obj, machine_moto_stop);

STATIC mp_obj_t machine_moto_run(mp_obj_t self_in) {
    machine_moto_obj_t *self = MP_OBJ_TO_PTR(self_in);
    machine_moto_pwm_obj_t *self_pina = &(self->pina);
    machine_moto_pwm_obj_t *self_pinb = &(self->pinb);
    mp_map_t kw_args;
    mp_obj_t ** v = &self->speed;
    // mp_map_init_fixed_table(&kw_args, 0, v);
    if (self->speed >= 0 && self->speed <= 100) {
        mp_machine_moto_duty((machine_moto_pwm_obj_t *)(self_pina), self->speed);
        mp_machine_moto_duty((machine_moto_pwm_obj_t *)(self_pinb), self->speed);
        mp_machine_moto_deinit((machine_moto_pwm_obj_t *)(self_pina));
    }else if (self->speed < 0 && self->speed >= -100) {
        mp_machine_moto_duty((machine_moto_pwm_obj_t *)(self_pina), self->speed);
        mp_machine_moto_duty((machine_moto_pwm_obj_t *)(self_pinb), self->speed);
        mp_machine_moto_deinit((machine_moto_pwm_obj_t *)(self_pinb));
    }else {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("speed is -100 ~ 100"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_moto_run_obj, machine_moto_run);

STATIC mp_obj_t machine_moto_speed(mp_obj_t self_in, int speed) {
    machine_moto_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->speed = mp_obj_get_int(speed);
    // printf("self->speed %d\n",self->speed);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(machine_moto_speed_obj, machine_moto_speed);

STATIC const mp_rom_map_elem_t machine_moto_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&machine_moto_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_run), MP_ROM_PTR(&machine_moto_run_obj) },
    { MP_ROM_QSTR(MP_QSTR_speed), MP_ROM_PTR(&machine_moto_speed_obj) },
};

STATIC MP_DEFINE_CONST_DICT(machine_moto_locals_dict, machine_moto_locals_dict_table);


const mp_obj_type_t machine_moto_type = {
    { &mp_type_type },
    .name = MP_QSTR_MOTO,
    .make_new = mp_machine_moto_make_new,
    .locals_dict = (mp_obj_t)&machine_moto_locals_dict,
};
