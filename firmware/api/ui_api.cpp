#include "ui_api.h"
#include "klippy.h"
#include <string>
#include "debug.h"
#include "Define_config_path.h"
#include "adxl345.h"
#include "resonance_tester.h"
#include "pid_calibrate.h"
#include "extruder.h"
#include "gcode_move.h"
#include "toolhead.h"

#include "ui_api.h"
#include "toolhead.h"
#include "mcu.h"
#include "pins.h"
#include "msgproto.h"
#include "heaters.h"
#include "fan.h"
#include "dspMemOps.h"
#include "gcode.h"
#include "gcode_move.h"
#include "force_move.h"
#include "extruder.h"
#include "heater_bed.h"
#include "canbus_ids.h"
#include "probe.h"
#include "tuning_tower.h"
#include "BLTouch.h"
#include "verify_heater.h"
#include "pid_calibrate.h"
#include "reactor.h"
#include "input_shaper.h"
#include "adxl345.h"
#include "bed_mesh.h"
#include "pause_resume.h"
#include "buttons.h"
#include "idle_timeout.h"
#include "heater_fan.h"
#include "webhooks.h"
#include "print_stats.h"
#include "virtual_sdcard.h"
#include "query_adc.h"
#include "query_endstops.h"
#include "statistics.h"
#include "fan_generic.h"
#include "resonance_tester.h"
#include "gcode_arcs.h"
#include "printer_para.h"
#include "../devices/serial_debug.h"
#include "net.h"
#include "wifi.h"
#include "auto_leveling.h"
#include "tmc2209.h"
#include "controller_fan.h"

api_cb ui_cb[100];

volatile bool print_gcode;
std::string print_file_path;

//===========================print api===================================================

void api_get_resonance_freq_cb(void *arg1, void *arg2, void *arg3, void *arg4)
{
    char *shaper_type_x = (char *)arg1;
    strcpy(shaper_type_x, Printer::GetInstance()->m_pconfig->GetString("input_shaper", "shaper_type_x", "").c_str());
    double *shaper_freq_x = (double *)arg2;
    *shaper_freq_x = Printer::GetInstance()->m_pconfig->GetDouble("input_shaper", "shaper_freq_x", 0);
    char *shaper_type_y = (char *)arg3;
    strcpy(shaper_type_y, Printer::GetInstance()->m_pconfig->GetString("input_shaper", "shaper_type_y", "").c_str());
    double *shaper_freq_y = (double *)arg4;
    *shaper_freq_y = Printer::GetInstance()->m_pconfig->GetDouble("input_shaper", "shaper_freq_y", 0);
}

void api_set_resonance_freq_cb(void *arg1, void *arg2, void *arg3, void *arg4)
{
    char *shaper_type_x = (char *)arg1;
    Printer::GetInstance()->m_pconfig->SetValue("input_shaper", "shaper_type_x", std::string(shaper_type_x));
    double *shaper_freq_x = (double *)arg2;
    Printer::GetInstance()->m_pconfig->SetDouble("input_shaper", "shaper_freq_x", *shaper_freq_x);
    char *shaper_type_y = (char *)arg3;
    Printer::GetInstance()->m_pconfig->GetString("input_shaper", "shaper_type_y", std::string(shaper_type_y));
    double *shaper_freq_y = (double *)arg4;
    Printer::GetInstance()->m_pconfig->SetDouble("input_shaper", "shaper_freq_y", *shaper_freq_y);
}

// void api_get_resonance_y_freq_cb(void *arg1 = NULL, void *arg2 = NULL)
// {
//     char *shaper_type = (char*) arg1;
//     strcpy(shaper_type, Printer::GetInstance()->m_pconfig->GetString("input_shaper", "shaper_type_y", "").c_str());
//     double *shaper_freq = (double *) arg2;
//     *shaper_freq = Printer::GetInstance()->m_pconfig->GetDouble("input_shaper", "shaper_freq_y", 0);
// }

void api_get_pid_value_cb(void *arg1, void *arg2, void *arg3)
{
    double *Kp = (double *)arg1;
    *Kp = Printer::GetInstance()->m_pconfig->GetDouble("extruder", "pid_Kp", 0);
    double *Ki = (double *)arg2;
    *Ki = Printer::GetInstance()->m_pconfig->GetDouble("extruder", "pid_Ki", 0);
    double *Kd = (double *)arg3;
    *Kd = Printer::GetInstance()->m_pconfig->GetDouble("extruder", "pid_Kd", 0);
}

void get_adxl_hal_exception_flag(void *arg1, void *arg2)
{
    std::string adxl_name((char *)arg1);
    int *flag = (int *)arg2;
    for (auto adxl : Printer::GetInstance()->m_adxl345s)
    {
        if (adxl.second->m_name == adxl_name)
            *flag = adxl.second->m_hal_exception_flag;
    }
}

void set_adxl_hal_exception_flag(void *arg1, void *arg2)
{
    std::string adxl_name((char *)arg1);
    int *flag = (int *)arg2;
    for (auto adxl : Printer::GetInstance()->m_adxl345s)
    {
        if (adxl.second->m_name == adxl_name)
            adxl.second->m_hal_exception_flag = *flag;
    }
}

void get_adxl_name(char **adxl_names)
{
    int index = 0;
    for (auto adxl : Printer::GetInstance()->m_adxl345s)
        adxl_names[index++] = (char *)adxl.second->m_name.c_str();
}

static void get_check_resonance_test_flag(void *arg = NULL)
{
    bool *flag = (bool *)arg;
    *flag = Printer::GetInstance()->m_resonance_tester->m_test_finish;
}

static void get_check_pid_calibrate_flag(void *arg = NULL)
{
    bool *flag = (bool *)arg;
    *flag = Printer::GetInstance()->m_pid_calibrate->m_finish_flag;
}

static void get_check_flow_calibration_flag(void *arg = NULL)
{
    bool *flag = (bool *)arg;
    *flag = Printer::GetInstance()->m_printer_extruder->m_flow_calibration;
}

static void set_check_resonance_test_flag(void *arg = NULL)
{
    bool *flag = (bool *)arg;
    Printer::GetInstance()->m_resonance_tester->m_test_finish = *flag;
}

static void set_check_pid_calibrate_flag(void *arg = NULL)
{
    bool *flag = (bool *)arg;
    Printer::GetInstance()->m_pid_calibrate->m_finish_flag = *flag;
}

static void set_check_flow_calibration_flag(void *arg = NULL)
{
    int *flag = (int *)arg;
    Printer::GetInstance()->m_printer_extruder->m_flow_calibration = *flag;
}

static void get_pa_val(void *arg = NULL)
{
    double *val = (double *)arg;
    *val = Printer::GetInstance()->m_printer_extruder->m_pressure_advance;
}

static void save_pa_val(void *arg = NULL) //-保存流量校准数据--
{
    double *val = (double *)arg;
    Printer::GetInstance()->m_pconfig->SetDouble("extruder", "pressure_advance", *val);
    char cmd[MANUAL_COMMAND_MAX_LENGTH];
    sprintf(cmd, "SET_PRESSURE_ADVANCE ADVANCE=%.2f", *val);
    manual_control_sq.push(cmd);
    Printer::GetInstance()->manual_control_signal();

    // Printer::GetInstance()->m_printer_extruder->set_pressure_advance(*val, 0);
    Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
}

static void get_line_length(void *arg = NULL)
{
    double *val = (double *)arg;
    *val = Printer::GetInstance()->m_pconfig->GetDouble("flow_calibration", "line_length", 150);
}

static void get_line_space(void *arg = NULL)
{
    double *val = (double *)arg;
    *val = Printer::GetInstance()->m_pconfig->GetDouble("flow_calibration", "line_space", 5);
}

static void set_line_length(void *arg = NULL)
{
    double *val = (double *)arg;
    Printer::GetInstance()->m_pconfig->SetDouble("flow_calibration", "line_length", *val);
}

static void set_line_space(void *arg = NULL)
{
    double *val = (double *)arg;
    Printer::GetInstance()->m_pconfig->SetDouble("flow_calibration", "line_space", *val);
}

static void write_ini(void *arg = NULL)
{
    Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
    // Printer::GetInstance()->m_unmodifiable_cfg->WriteIni(UNMODIFIABLE_CFG_PATH);
}

static void api_start_print(void *arg = NULL)
{
    std::string file_path((char *)arg);
    print_file_path = file_path;
    // std::cout << "path : " << print_file_path << std::endl;
    std::string print_cmd = "wake up thread";
    // print_gcode = true;
    manual_control_sq.push(print_cmd);
    manual_control_sq.push(print_file_path);
    Printer::GetInstance()->manual_control_signal();
    printf("api_start_print : %s  \n", print_cmd.c_str());
    // GCodeDispatch gcode_dispatch;
    // gcode_dispatch.processCommands(file_path);
}

static void api_stop_print(void *arg = NULL)
{
    std::string file_path((char *)arg);
    print_file_path = file_path;
    std::string print_cmd = "wake up thread";
    manual_control_sq.push(print_cmd);
    Printer::GetInstance()->manual_control_signal();
    printf("api_stop_print : %s  \n", print_cmd.c_str());
}

static void api_command_control(void *arg)
{
    std::string control_line((char *)arg);
    manual_control_sq.push(control_line);
    Printer::GetInstance()->manual_control_signal();
    printf("api_manual_control : %s  \n", control_line.c_str());
}

static void api_command_queue_clear(void *arg = NULL)
{
    manual_control_sq.clear();
}

static void api_highest_priority_cmd_cb(void *arg)
{
    std::string control_line((char *)arg);
    highest_priority_cmd_sq.push(control_line);
    Printer::GetInstance()->highest_priority_control_signal();
    printf("api_manual_control : %s  \n", control_line.c_str());
}

static void api_fan_cmd_cb(void *arg)
{
    std::string control_line((char *)arg);
    fan_cmd_sq.push(control_line);
    Printer::GetInstance()->fan_cmd_control_signal();
    // printf("api_manual_control : %s  \n",control_line.c_str());
}

static void api_set_print_speed(void *arg)
{
    int *value = (int *)arg;
    double val = *value / (60. * 100.);
    Printer::GetInstance()->m_gcode_move->m_speed = Printer::GetInstance()->m_gcode_move->get_gcode_speed() * val;
    Printer::GetInstance()->m_gcode_move->m_speed_factor = val;

    if (Printer::GetInstance()->m_print_stats->get_status(get_monotonic(), NULL).state == PRINT_STATS_STATE_PAUSED)
    {
        auto iter = Printer::GetInstance()->m_gcode_move->saved_states.find("PAUSE_STATE");
        if (iter != Printer::GetInstance()->m_gcode_move->saved_states.end())
        {
            iter->second.speed = iter->second.speed / iter->second.speed_factor * val;
            iter->second.speed_factor = val;
        }
    }
}

static void api_set_cur_Zslect_cb(void *arg)
{
    int *cur_Zslect = (int *)arg;
    GAM_DEBUG_printf("--%d %d-----\n", *cur_Zslect, Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->cur_stepper_slect);
    if (Printer::GetInstance()->m_tool_head->m_kin->m_rails.size() > 2)
        Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->cur_stepper_slect = *cur_Zslect;
}

static void api_get_print_speed(void *arg)
{
    double *speed_multi = (double *)arg;
    *speed_multi = Printer::GetInstance()->m_gcode_move->get_gcode_speed_override();
}

static void api_get_print_actual_speed(void *arg)
{
    double *speed = (double *)arg;
    *speed = Printer::GetInstance()->m_gcode_move->get_gcode_speed();
}

static void api_get_print_size(void *arg)
{
    int *size = (int *)arg;
    *size = Printer::GetInstance()->m_virtual_sdcard->m_file_size;
}

static void api_get_alread_print_size(void *arg)
{
    int *size = (int *)arg;
    *size = Printer::GetInstance()->m_virtual_sdcard->m_file_position;
}

static void api_get_already_print_time(void *arg)
{
    double *time = (double *)arg;
    // *time = Printer::GetInstance()->m_print_stats->get_print_duration();
    *time = Printer::GetInstance()->m_print_stats->get_status(get_monotonic(), NULL).print_duration;
}

static void api_get_total_layer(void *arg)
{
    int *total_layer = (int *)arg;
    *total_layer = Printer::GetInstance()->m_print_stats->get_status(get_monotonic(), NULL).total_layers;
}

static void api_get_current_layer(void *arg)
{
    int *current_layer = (int *)arg;
    *current_layer = Printer::GetInstance()->m_print_stats->get_status(get_monotonic(), NULL).current_layer;
}

static void api_get_print_pos(void *arg)
{
    double *pos_arr = (double *)arg;
    std::vector<double> pos = Printer::GetInstance()->m_tool_head->m_commanded_pos;
    pos_arr[0] = pos[0];
    pos_arr[1] = pos[1];
    pos_arr[2] = pos[2];
    pos_arr[3] = pos[3];
}

static void api_get_stepper_x_state(void *arg)
{
    bool *state = (bool *)arg;
    *state = Printer::GetInstance()->m_stepper_enable->get_stepper_state("stepper_x");
}

static void api_get_stepper_y_state(void *arg)
{
    bool *state = (bool *)arg;
    *state = Printer::GetInstance()->m_stepper_enable->get_stepper_state("stepper_y");
}

static void api_get_stepper_z_state(void *arg)
{
    bool *state = (bool *)arg;
    *state = Printer::GetInstance()->m_stepper_enable->get_stepper_state("stepper_z");
}

static void api_get_stepper_e_state(void *arg)
{
    bool *state = (bool *)arg;
    *state = Printer::GetInstance()->m_stepper_enable->get_stepper_state("stepper_z");
}

static void register_check_resonance_test_cb()
{
    ui_cb[check_resonance_test_cb] = get_check_resonance_test_flag;
}

static void register_check_pid_calibrate_cb()
{
    ui_cb[check_pid_calibrate_cb] = get_check_pid_calibrate_flag;
}

static void register_check_flow_calibration_cb()
{
    ui_cb[check_flow_calibration_cb] = get_check_flow_calibration_flag;
}

static void register_get_pa_val_cb()
{
    ui_cb[get_pa_val_cb] = get_pa_val;
}

static void register_save_pa_val_cb()
{
    ui_cb[save_pa_cb] = save_pa_val;
}

static void register_get_line_length_cb()
{
    ui_cb[get_line_length_cb] = get_line_length;
}

static void register_get_line_space_cb()
{
    ui_cb[get_line_space_cb] = get_line_space;
}

static void register_set_resonance_test_cb()
{
    ui_cb[set_resonance_test_cb] = set_check_resonance_test_flag;
}

static void register_pid_calibrate_cb()
{
    ui_cb[set_pid_calibrate_cb] = set_check_pid_calibrate_flag;
}

static void register_set_flow_calibration_cb()
{
    ui_cb[set_flow_calibration_cb] = set_check_flow_calibration_flag;
}

static void register_print_start_cb()
{
    ui_cb[print_start_cb] = api_start_print;
}

static void register_print_stop_cb()
{
    ui_cb[print_stop_cb] = api_stop_print;
}

static void register_manual_control_cb()
{
    ui_cb[manual_control_cb] = api_command_control;
}

static void register_manual_control_clear_cb()
{
    ui_cb[command_queue_clear_cb] = api_command_queue_clear;
}

static void register_set_print_speed_cb()
{
    ui_cb[set_print_speed_cb] = api_set_print_speed;
}

static void register_set_cur_Zslect_cb()
{
    ui_cb[set_cur_Zslect_cb] = api_set_cur_Zslect_cb;
}

static void register_get_print_speed_cb()
{
    ui_cb[get_print_speed_cb] = api_get_print_speed;
}

static void register_get_print_size_cb()
{
    ui_cb[get_print_size_cb] = api_get_print_size;
}

static void register_get_alread_print_size_cb()
{
    ui_cb[get_alread_print_size_cb] = api_get_alread_print_size;
}

static void register_get_alread_print_time_cb()
{
    ui_cb[get_alread_print_time_cb] = api_get_already_print_time;
}

static void register_get_total_layer_cb()
{
    ui_cb[get_total_layer_cb] = api_get_total_layer;
}

static void register_get_current_layer_cb()
{
    ui_cb[get_current_layer_cb] = api_get_current_layer;
}

static void register_get_print_pos_cb()
{
    ui_cb[get_print_pos_cb] = api_get_print_pos;
}

//============================heat api===========================================

static void api_extruder1_heat_control(void *arg)
{
    int temp = *((int *)arg);
    // char heat_cmd[MANUAL_COMMAND_MAX_LENGTH];
    // sprintf(heat_cmd, "M104 S%d", temp);
    // ui_cb[manual_control_cb](heat_cmd);
    Printer::GetInstance()->m_printer_extruder->m_heater->m_lock.lock();
    Printer::GetInstance()->m_printer_extruder->m_heater->m_target_temp = (double)temp;
    Printer::GetInstance()->m_printer_extruder->m_heater->m_lock.unlock();
}

static void api_extruder2_heat_control(void *arg)
{
}

static void api_bed_heat_control(void *arg)
{
    int temp = *((int *)arg);
    // char heat_cmd[MANUAL_COMMAND_MAX_LENGTH];
    // sprintf(heat_cmd, "M140 S%d", temp);
    // ui_cb[manual_control_cb](heat_cmd);
    Printer::GetInstance()->m_bed_heater->m_heater->m_lock.lock();
    Printer::GetInstance()->m_bed_heater->m_heater->m_target_temp = (double)temp;
    Printer::GetInstance()->m_bed_heater->m_heater->m_lock.unlock();
}
static void api_set_fan_speed(void *arg)
{
    int val = *((int *)arg);
    // Printer::GetInstance()->m_printer_fan->m_fan->m_lock.lock();
    Printer::GetInstance()->m_printer_fan->m_fan->set_speed_from_command(val / 100.);
    // Printer::GetInstance()->m_printer_fan->m_fan->m_lock.unlock();
}

static void api_get_extruder1_current_temp(void *arg)
{
    double *temp = (double *)arg;
    Printer::GetInstance()->m_printer_extruder->m_heater->m_lock.lock();
    *temp = Printer::GetInstance()->m_printer_extruder->m_heater->m_smoothed_temp;
    Printer::GetInstance()->m_printer_extruder->m_heater->m_lock.unlock();
}

static void api_get_extruder2_current_temp(void *arg)
{
}

static void api_get_bed_current_temp(void *arg)
{
    double *temp = (double *)arg;
    Printer::GetInstance()->m_bed_heater->m_heater->m_lock.lock();
    *temp = Printer::GetInstance()->m_bed_heater->m_heater->m_smoothed_temp;
    Printer::GetInstance()->m_bed_heater->m_heater->m_lock.unlock();
}

static void api_get_box_current_temp(void *arg)
{
}

static void api_get_box_target_temp(void *arg)
{
}

static void
api_get_extruder1_target_temp(void *arg)
{
    double *temp = (double *)arg;
    Printer::GetInstance()->m_printer_extruder->m_heater->m_lock.lock();
    *temp = Printer::GetInstance()->m_printer_extruder->m_heater->m_target_temp;
    Printer::GetInstance()->m_printer_extruder->m_heater->m_lock.unlock();
}

static void api_get_extruder2_target_temp(void *arg)
{
}

static void api_get_bed_target_temp(void *arg)
{
    double *temp = (double *)arg;
    Printer::GetInstance()->m_bed_heater->m_heater->m_lock.lock();
    if (Printer::GetInstance()->m_bed_heater) //
    {
        *temp = Printer::GetInstance()->m_bed_heater->m_heater->m_target_temp;
    }
    else
    {
        *temp = 0.0;
    }
    Printer::GetInstance()->m_bed_heater->m_heater->m_lock.unlock();
}

static void register_extruder1_heat_cb()
{
    ui_cb[extruder1_heat_cb] = api_extruder1_heat_control;
}

static void register_extruder2_heat_cb()
{
    ui_cb[extruder2_heat_cb] = api_extruder2_heat_control;
}

static void register_bed_heat_cb()
{
    ui_cb[bed_heat_cb] = api_bed_heat_control;
}

static void register_get_extruder1_target_temp_cb()
{
    ui_cb[get_extruder1_target_temp_cb] = api_get_extruder1_target_temp;
}

static void register_get_extruder2_target_temp_cb()
{
    ui_cb[get_extruder2_target_temp_cb] = api_get_extruder2_target_temp;
}

static void register_get_bed_target_temp_cb()
{
    ui_cb[get_bed_target_temp_cb] = api_get_bed_target_temp;
}

static void register_get_extruder1_current_temp_cb()
{
    ui_cb[get_extruder1_curent_temp_cb] = api_get_extruder1_current_temp;
}

static void register_get_extruder2_current_temp_cb()
{
    ui_cb[get_extruder2_curent_temp_cb] = api_get_extruder2_current_temp;
}

static void register_get_bed_current_temp_cb()
{
    ui_cb[get_bed_curent_temp_cb] = api_get_bed_current_temp;
}

static void register_get_box_current_temp_cb()
{
    ui_cb[get_box_curent_temp_cb] = api_get_box_current_temp;
}

static void register_get_box_target_temp_cb()
{
    ui_cb[get_box_target_temp_cb] = api_get_box_target_temp;
}

//====================================set get fan api=============================================

static void api_printer_fan_control(void *arg)
{
    std::string control_line((char *)arg);
    manual_control_sq.push(control_line);
    Printer::GetInstance()->manual_control_signal();
}

static void api_fan0_control(void *arg)
{
    if (Printer::GetInstance()->m_generic_fans.size() >= 1)
    {
        std::string control_line((char *)arg);
        std::string fan_name = Printer::GetInstance()->m_generic_fans[0]->m_fan_name;
        manual_control_sq.push(fan_name + control_line);
        Printer::GetInstance()->manual_control_signal();
    }
    //  GAM_DEBUG_printf("api_command_control : %s  \n",control_line.c_str());
}

static void api_fan1_control(void *arg)
{
    if (Printer::GetInstance()->m_generic_fans.size() >= 2)
    {
        std::string control_line((char *)arg);
        std::string fan_name = Printer::GetInstance()->m_generic_fans[1]->m_fan_name;
        manual_control_sq.push(fan_name + control_line);
        Printer::GetInstance()->manual_control_signal();
    }
}

static void api_fan2_control(void *arg)
{
    if (Printer::GetInstance()->m_generic_fans.size() >= 3)
    {
        std::string control_line((char *)arg);
        std::string fan_name = Printer::GetInstance()->m_generic_fans[2]->m_fan_name;
        manual_control_sq.push(fan_name + control_line);
        Printer::GetInstance()->manual_control_signal();
    }
}

static void api_set_printer_fan_speed(void *arg)
{
    int *speed = (int *)arg;
    Printer::GetInstance()->m_printer_fan->m_fan->m_current_speed = *speed;
}

static void api_set_fan0_speed(void *arg)
{
    if (Printer::GetInstance()->m_generic_fans.size() >= 1)
    {
        double *speed = (double *)arg;
        Printer::GetInstance()->m_generic_fans[0]->m_fan->m_current_speed = *speed;
    }
}

static void api_set_fan1_speed(void *arg)
{
    if (Printer::GetInstance()->m_generic_fans.size() >= 2)
    {
        double *speed = (double *)arg;
        Printer::GetInstance()->m_generic_fans[1]->m_fan->m_current_speed = *speed;
    }
}

static void api_set_fan2_speed(void *arg)
{
    if (Printer::GetInstance()->m_generic_fans.size() >= 3)
    {
        double *speed = (double *)arg;
        Printer::GetInstance()->m_generic_fans[2]->m_fan->m_current_speed = *speed;
    }
}

static void api_get_printer_fan_speed(void *arg)
{
    int *speed = (int *)arg;
    *speed = (int)(Printer::GetInstance()->m_printer_fan->m_fan->m_current_speed * 100 + 0.5);
}

static void api_get_fan0_speed(void *arg)
{
    if (Printer::GetInstance()->m_generic_fans.size() >= 1)
    {
        double *speed = (double *)arg;
        *speed = Printer::GetInstance()->m_generic_fans[0]->m_fan->m_current_speed * 100;
    }
}

static void api_get_fan1_speed(void *arg)
{
    if (Printer::GetInstance()->m_generic_fans.size() >= 2)
    {
        double *speed = (double *)arg;
        *speed = Printer::GetInstance()->m_generic_fans[1]->m_fan->m_current_speed;
    }
}

static void api_get_fan2_speed(void *arg)
{
    if (Printer::GetInstance()->m_generic_fans.size() >= 3)
    {
        double *speed = (double *)arg;
        *speed = Printer::GetInstance()->m_generic_fans[2]->m_fan->m_current_speed;
    }
}

static void api_get_print_time_cb(void *arg)
{
    double *time = (double *)arg;
    *time = Printer::GetInstance()->m_print_stats->get_print_duration();
}

static void api_get_z_offset_cb(void *arg)
{
    double *z_offset = (double *)arg;
    
    *z_offset = Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop - Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop_extra - Printer::GetInstance()->m_gcode_move->m_base_position[2] - Printer::GetInstance()->m_strain_gauge->m_cfg->m_fix_z_offset;
}

static void api_set_z_offset_cb(void *arg)
{
    double *z_offset = (double *)arg;
    char cmd[50];
    std::cout << "api_set_z_offset_cb z_offset: " << *z_offset << std::endl;
    double z_offset_param = 0. - (*z_offset - (Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop - Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop_extra - Printer::GetInstance()->m_strain_gauge->m_cfg->m_fix_z_offset));
    std::cout << "m_position_endstop: " << Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop  << " m_position_endstop_extra:"<< Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop_extra  << " m_fix_z_offset:" << Printer::GetInstance()->m_strain_gauge->m_cfg->m_fix_z_offset << std::endl;
    std::cout << "api_set_z_offset_cb z_offset_param: " << z_offset_param<< std::endl;
    sprintf(cmd, "SET_GCODE_OFFSET Z=%.3f MOVE=1 MOVE_SPEED=5.0", z_offset_param);
    manual_control_sq.push(cmd);
    Printer::GetInstance()->manual_control_signal();
}

static void api_save_z_offset_cb(void *arg)
{
    double *z_offset = (double *)arg;
    char cmd[20];
    sprintf(cmd, "M8233 S%.3f P0", (0.0 - *z_offset));
    manual_control_sq.push(cmd);
    // sprintf(cmd, "M8800");
    // manual_control_sq.push(cmd);
    Printer::GetInstance()->manual_control_signal();
}

static void regisetr_save_z_offset_cb()
{
    ui_cb[save_z_offset_cb] = api_save_z_offset_cb;
}

static void regisetr_get_z_offset_cb()
{
    ui_cb[get_z_offset_cb] = api_get_z_offset_cb;
}

static void regisetr_set_z_offset_cb()
{
    ui_cb[set_z_offset_cb] = api_set_z_offset_cb;
}

static void regisetr_get_print_time_cb()
{
    ui_cb[get_print_time] = api_get_print_time_cb;
}

static void register_api_printer_fan_cb()
{
    ui_cb[printer_fan_cb] = api_printer_fan_control;
}

static void register_api_fan0_cb()
{
    ui_cb[fan0_cb] = api_fan0_control;
}

static void register_api_fan1_cb()
{
    ui_cb[fan1_cb] = api_fan1_control;
}

static void register_api_fan2_cb()
{
    ui_cb[fan2_cb] = api_fan2_control;
}

static void register_get_api_printer_fan_speed_cb()
{
    ui_cb[get_printer_fan_speed_cb] = api_get_printer_fan_speed;
}

static void register_get_api_fan0_speed_cb()
{
    ui_cb[get_fan0_speed_cb] = api_get_fan0_speed;
}

static void register_get_api_fan1_speed_cb()
{
    ui_cb[get_fan1_speed_cb] = api_get_fan1_speed;
}

static void register_get_api_fan2_speed_cb()
{
    ui_cb[get_fan2_speed_cb] = api_get_fan2_speed;
}

static void register_set_api_printer_fan_speed_cb()
{
    ui_cb[set_printer_fan_speed_cb] = api_set_printer_fan_speed;
}

static void register_set_api_fan0_speed_cb()
{
    ui_cb[set_fan0_speed_cb] = api_set_fan0_speed;
}

static void register_set_api_fan1_speed_cb()
{
    ui_cb[set_fan1_speed_cb] = api_set_fan1_speed;
}

static void register_set_api_fan2_speed_cb()
{
    ui_cb[set_fan2_speed_cb] = api_set_fan2_speed;
}

static void api_get_pos_info(void *arg)
{
    std::vector<double> currentpos = Printer::GetInstance()->m_tool_head->m_commanded_pos;
    *(std::vector<double> *)arg = currentpos;
}
static void register_get_pos_info_cb()
{
    ui_cb[get_pos_info_cb] = api_get_pos_info;
}
static void api_get_print_state(void *arg)
{
    print_stats_state_s state = Printer::GetInstance()->m_print_stats->m_print_stats.state;
    std::cout << state << std::endl;
    if (state == PRINT_STATS_STATE_COMPLETED || state == PRINT_STATS_STATE_STANDBY)
    {
        *(bool *)arg = true;
    }
    else
    {
        *(bool *)arg = false;
    }
}

static void api_is_print_complete(void *arg)
{
    print_stats_state_s state = Printer::GetInstance()->m_print_stats->m_print_stats.state;
    if (state == PRINT_STATS_STATE_COMPLETED)
    {
        *(bool *)arg = true;
        Printer::GetInstance()->m_print_stats->reset();
    }
    else
    {
        *(bool *)arg = false;
    }
}

static void register_get_is_print_complete_cb()
{
    ui_cb[get_is_print_complete_cb] = api_is_print_complete;
}

static void register_get_print_state_cb()
{
    ui_cb[get_print_state_cb] = api_get_print_state;
}

static void register_get_stepper_x_state_cb()
{
    ui_cb[get_stepper_x_state] = api_get_stepper_x_state;
}

static void register_get_stepper_y_state_cb()
{
    ui_cb[get_stepper_y_state] = api_get_stepper_y_state;
}

static void register_get_stepper_z_state_cb()
{
    ui_cb[get_stepper_z_state] = api_get_stepper_z_state;
}

static void register_get_stepper_e_state_cb()
{
    ui_cb[get_stepper_e_state] = api_get_stepper_e_state;
}

static void register_set_line_length_cb()
{
    ui_cb[set_line_length_cb] = set_line_length;
}

static void register_set_line_space_cb()
{
    ui_cb[set_line_space_cb] = set_line_space;
}

static void register_write_ini_cb()
{
    ui_cb[write_ini_cb] = write_ini;
}

static void api_position_calibration(void *arg)
{
    std::map<std::string, std::string> kin_status = Printer::GetInstance()->m_tool_head->get_status(get_monotonic());
    if (kin_status["homed_axes"].find("x") == std::string::npos || kin_status["homed_axes"].find("y") == std::string::npos || kin_status["homed_axes"].find("z") == std::string::npos)
    {
        manual_control_sq.push("G28");
    }
    manual_control_sq.push("G90");
    manual_control_sq.push("G0 Z10 F360");
    manual_control_sq.push("M211 S0");
    char cmd[MANUAL_COMMAND_MAX_LENGTH];
    sprintf(cmd, "G0 X%.2f Y%.2f F%.2f", Printer::GetInstance()->m_auto_leveling->target_spot_x, Printer::GetInstance()->m_auto_leveling->target_spot_y, Printer::GetInstance()->m_auto_leveling->move_speed);
    manual_control_sq.push(cmd); //---XY移动校准模块，速度可配置---
    manual_control_sq.push("PROBE");
    manual_control_sq.push("G0 Z10 F360");
    manual_control_sq.push("G91");
    Printer::GetInstance()->manual_control_signal();
}

static void api_position_calibration_save_x(void *arg)
{
    double *x = (double *)arg;
    Printer::GetInstance()->m_auto_leveling->target_spot_x = *x;
    std::cout << "x = " << Printer::GetInstance()->m_auto_leveling->target_spot_x << std::endl;
}

static void api_position_calibration_save_y(void *arg)
{
    double *y = (double *)arg;
    Printer::GetInstance()->m_auto_leveling->target_spot_y = *y;
    std::cout << "y = " << Printer::GetInstance()->m_auto_leveling->target_spot_y << std::endl;
}

static void api_get_target_spot_x(void *arg)
{
    double *x = (double *)arg;
    *x = Printer::GetInstance()->m_auto_leveling->target_spot_x;
}

static void api_get_target_spot_y(void *arg)
{
    double *y = (double *)arg;
    *y = Printer::GetInstance()->m_auto_leveling->target_spot_y;
}

static void api_position_calibration_save(void *arg)
{
    Printer::GetInstance()->m_pconfig->SetDouble("auto_leveling", "target_spot_x", Printer::GetInstance()->m_auto_leveling->target_spot_x);
    Printer::GetInstance()->m_pconfig->SetDouble("auto_leveling", "target_spot_y", Printer::GetInstance()->m_auto_leveling->target_spot_y);
    // Printer::GetInstance()->m_unmodifiable_cfg->WriteIni(UNMODIFIABLE_CFG_PATH);
}

static void api_position_calibration_read(void *arg)
{
    Printer::GetInstance()->m_auto_leveling->target_spot_x = Printer::GetInstance()->m_pconfig->GetDouble("auto_leveling", "target_spot_x", Printer::GetInstance()->m_auto_leveling->target_spot_x);
    Printer::GetInstance()->m_auto_leveling->target_spot_y = Printer::GetInstance()->m_pconfig->GetDouble("auto_leveling", "target_spot_y", Printer::GetInstance()->m_auto_leveling->target_spot_y);
}

static void api_set_silent_mode(void *arg)
{
    bool *silent_mode = (bool *)arg;
    if (Printer::GetInstance()->m_communication_state)
    {
        Printer::GetInstance()->send_event("set_silent_mode", *silent_mode);
    }
}

static void api_get_printing_wait(void *arg)
{
    bool *wait = (bool *)arg;
    *wait = Printer::GetInstance()->m_pause_resume->m_wait;
}

static void api_set_printing_wait(void *arg)
{
    bool *wait = (bool *)arg;
    Printer::GetInstance()->m_pause_resume->m_wait = *wait;
}

static void api_stop_move(void *arg)
{
    Printer::GetInstance()->m_tool_head->stop_move();
}

static void api_stop_extrude(void *arg)
{
    Printer::GetInstance()->m_printer_extruder->load_filament_flag = true;
    Printer::GetInstance()->m_tool_head->stop_move();
}

static void api_reset_acceleration(void *arg)
{
    Printer::GetInstance()->m_gcode_io->single_command("M204 S%.2f", Printer::GetInstance()->m_tool_head->m_max_accel_limit);
}

static void api_get_print_status(void *arg)
{
    if (arg != NULL && Printer::GetInstance()->m_print_stats != nullptr)
    {
        print_stats_t status = Printer::GetInstance()->m_print_stats->get_status(get_monotonic(), NULL);
        memcpy(arg, &status, sizeof(print_stats_t));
    }
}

static void register_position_calibration_cb()
{
    ui_cb[position_calibration_cb] = api_position_calibration;
}

static void register_position_calibration_save_x_cb()
{
    ui_cb[position_calibration_save_x_cb] = api_position_calibration_save_x;
}

static void register_position_calibration_save_y_cb()
{
    ui_cb[position_calibration_save_y_cb] = api_position_calibration_save_y;
}

static void register_position_calibration_save_cb()
{
    ui_cb[position_calibration_save_cb] = api_position_calibration_save;
}

static void api_clear_print_stats(void *arg)
{
    Printer::GetInstance()->m_print_stats->reset();
}

void register_api()
{
    register_print_start_cb();
    register_print_stop_cb();
    register_manual_control_cb();
    register_manual_control_clear_cb();
    ui_cb[highest_priority_cmd_cb] = api_highest_priority_cmd_cb;
    register_set_print_speed_cb();
    register_get_print_speed_cb();
    ui_cb[get_print_actual_speed] = api_get_print_actual_speed;
    register_get_print_size_cb();
    register_get_print_state_cb();
    register_get_alread_print_time_cb();
    register_get_total_layer_cb();
    register_get_current_layer_cb();
    register_get_alread_print_size_cb();
    register_get_print_pos_cb();

    register_extruder1_heat_cb();
    register_extruder2_heat_cb();
    register_bed_heat_cb();
    register_get_extruder1_target_temp_cb();
    register_get_extruder2_target_temp_cb();
    register_get_bed_target_temp_cb();
    register_get_extruder1_current_temp_cb();
    register_get_extruder2_current_temp_cb();
    register_get_bed_current_temp_cb();
    register_get_box_current_temp_cb();
    register_get_box_target_temp_cb();

    register_api_printer_fan_cb();
    register_api_fan0_cb();
    register_api_fan1_cb();
    register_api_fan2_cb();
    register_get_api_printer_fan_speed_cb();
    register_get_api_fan0_speed_cb();
    register_get_api_fan1_speed_cb();
    register_get_api_fan2_speed_cb();
    register_set_api_printer_fan_speed_cb();
    register_set_api_fan0_speed_cb();
    register_set_api_fan1_speed_cb();
    register_set_api_fan2_speed_cb();

    register_get_pos_info_cb();
    register_get_is_print_complete_cb();
    register_set_cur_Zslect_cb();

    register_check_resonance_test_cb();
    register_check_pid_calibrate_cb();
    register_check_flow_calibration_cb();
    register_set_flow_calibration_cb();
    register_set_resonance_test_cb();
    register_pid_calibrate_cb();

    register_get_stepper_x_state_cb();
    register_get_stepper_y_state_cb();
    register_get_stepper_z_state_cb();
    register_get_stepper_e_state_cb();

    regisetr_get_print_time_cb();
    register_get_pa_val_cb();
    register_save_pa_val_cb();
    register_get_line_length_cb();
    register_get_line_space_cb();
    register_set_line_length_cb();
    register_set_line_space_cb();
    register_write_ini_cb();

    regisetr_set_z_offset_cb();
    regisetr_get_z_offset_cb();
    regisetr_save_z_offset_cb();

    register_position_calibration_cb();
    register_position_calibration_save_x_cb();
    register_position_calibration_save_y_cb();
    register_position_calibration_save_cb();

    ui_cb[get_printing_wait_cb] = api_get_printing_wait;
    ui_cb[set_printing_wait_cb] = api_set_printing_wait;

    ui_cb[stop_move_cb] = api_stop_move;
    ui_cb[stop_extrude_cb] = api_stop_extrude;
    ui_cb[set_fan_speed_cb] = api_fan_cmd_cb;
    ui_cb[position_calibration_read] = api_position_calibration_read;

    ui_cb[get_target_spot_x] = api_get_target_spot_x;
    ui_cb[get_target_spot_y] = api_get_target_spot_y;
    ui_cb[reset_acceleration_cb] = api_reset_acceleration;

    ui_cb[get_print_status] = api_get_print_status;
    ui_cb[set_silent_mode_cb] = api_set_silent_mode;
    ui_cb[clear_print_stats_cb] = api_clear_print_stats;
}