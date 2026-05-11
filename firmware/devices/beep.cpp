#include "beep.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#define BEEP_PWM_STR "4"

static bool beeper_enable = true;
static bool beep_on_status = false;
void *beep_process_task(void *arg);

int beep_on(bool argc)
{
    char cmd_str[100];
    beep_on_status = argc;
    memset(cmd_str, 0, sizeof(cmd_str));
    if (argc)
    {
        strcpy(cmd_str, "echo 1 > /sys/class/pwm/pwmchip0/pwm");
    }
    else
    {
        strcpy(cmd_str, "echo 0 > /sys/class/pwm/pwmchip0/pwm");
    }
    strcat(cmd_str, BEEP_PWM_STR);
    strcat(cmd_str, "/enable");

    int resB = system(cmd_str);
    // std::cout << "system run:"<< cmd_str << " resB: " << resB << std::endl;
    return resB;
}
int beepinit()
{
    char cmd_str[100];
    memset(cmd_str, 0, sizeof(cmd_str));
    beeper_enable = true;

    // echo 2000000 > /sys/class/pwm/pwmchip0/pwm4/period
    // echo 1000000 > /sys/class/pwm/pwmchip0/pwm4/duty_cycle
    // echo 1 > /sys/class/pwm/pwmchip0/pwm4/enable

    strcpy(cmd_str, "echo "); //
    strcat(cmd_str, BEEP_PWM_STR);
    strcat(cmd_str, " > /sys/class/pwm/pwmchip0/export");
    int resB = system(cmd_str);
    // std::cout << "system run:"<< cmd_str << " resB: " << resB << std::endl;
    memset(cmd_str, 0, sizeof(cmd_str));
    strcpy(cmd_str, "echo 400000 > /sys/class/pwm/pwmchip0/pwm");
    strcat(cmd_str, BEEP_PWM_STR);
    strcat(cmd_str, "/period");
    resB = system(cmd_str);
    // std::cout << "system run:"<< cmd_str << " resB: " << resB << std::endl;
    memset(cmd_str, 0, sizeof(cmd_str));
    strcpy(cmd_str, "echo 200000 > /sys/class/pwm/pwmchip0/pwm");
    strcat(cmd_str, BEEP_PWM_STR);
    strcat(cmd_str, "/duty_cycle");
    resB = system(cmd_str);
    // std::cout << "system run:"<< cmd_str << " resB: " << resB << std::endl;

    // 蜂鸣器
    pthread_t beep_process_tid;
    pthread_create(&beep_process_tid, NULL, beep_process_task, NULL);
    return 0;
}

void *beep_process_task(void *arg)
{
    while (1)
    {
        Beep_timer_handler();
        usleep(10000);
    }
}

static beeper_status_t beeper_status;
int beeper_cnt_10ms = 0;
int beeper_time_10ms = 0; // 蜂鸣器叫时间
int beeper_option_time_10ms = 0;
int beeper_cnt_times = 0;
int beeper_times = 0;

void Beep_timer_handler() //
{
    if (beeper_times)
    {
        // std::cout << "beeper_times : " << beeper_times << std::endl;
        // std::cout << "beeper_cnt_times : " << beeper_cnt_times << std::endl;
        if (beeper_time_10ms) // 蜂鸣器短叫或长叫
        {
            if (((beeper_status == BEEPER_SHORT_YET) && ((beeper_cnt_10ms % 30) == 0)) || ((beeper_status == BEEPER_LONG_YET) && ((beeper_cnt_10ms % 150) == 0)))
            {
                beep_on(!beep_on_status);
            }
            beeper_cnt_10ms++;
            if (beeper_cnt_10ms > beeper_time_10ms)
            {
                // freqnum = 0;
                // freqpeatnum = 0;
                beeper_time_10ms = beeper_cnt_10ms = 0;
                beep_on(false);
                beeper_cnt_times++;
                if(beeper_cnt_times >= beeper_times)
                {
                    beeper_cnt_times = beeper_times = 0;
                }
                else
                {
                    beeper_time_10ms = beeper_option_time_10ms;
                }
            }
        }
    }
}

void set_beeper_status(beeper_status_t status, int option_time_10ms, bool force)
{
    if (beeper_enable)
    {
        beeper_time_10ms = 0;
        if (BEEPER_YET_ALWAYS == status)
        {
            beep_on(true);
        }
        else if (BEEPER_STOP == status)
        {
            beep_on(false);
        }
        else
        {
            beeper_status = status;
            beeper_time_10ms = option_time_10ms;
            beeper_times = 1; 
            beeper_option_time_10ms = option_time_10ms;
        }
        beeper_cnt_10ms = 0;
    }
}

void set_beeper_status_repeat(beeper_status_t status, int option_time_10ms, int times, bool force)
{
    if (beeper_enable)
    {
        beeper_time_10ms = 0;
        if (BEEPER_YET_ALWAYS == status)
        {
            beep_on(true);
        }
        else if (BEEPER_STOP == status)
        {
            beep_on(false);
        }
        else
        {
            beeper_status = status;
            beeper_time_10ms = option_time_10ms;
            beeper_times = times;
            beeper_option_time_10ms = option_time_10ms;
        }
        beeper_cnt_10ms = 0;
    }
}