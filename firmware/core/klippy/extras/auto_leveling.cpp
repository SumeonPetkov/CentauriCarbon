#include "auto_leveling.h"
#include "probe.h"
#include "klippy.h"
#include "my_string.h"
#include "config.h"

#define LOG_TAG "auto_leveling"
#undef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#include "log.h"

#define AUTO_LEVELING_EHSM_DEBUG 0 // DEBUG下不等待加温和降温，直接走完流程，方便调试。
#define AUTO_LEVELING_EVENT_QUEUE_SIZE 16
enum
{
    AUTO_LEVELING_EVENT_START = UTILS_EHSM_USER_EVENT,
    AUTO_LEVELING_EVENT_START_HOMING,
    AUTO_LEVELING_EVENT_HOMING_Z_SUCC,
    AUTO_LEVELING_EVENT_HOMING_Y_SUCC,
    AUTO_LEVELING_EVENT_HOMING_SEED_LIMIT,
    AUTO_LEVELING_EVENT_HOMING_OUT_LIMIT,
    AUTO_LEVELING_EVENT_PROBE_SUCC,
    AUTO_LEVELING_EVENT_FALT
};
typedef struct auto_leveling_tag
{
    utils_ehsm_t ehsm;
    utils_ehsm_state_t idle;
    utils_ehsm_state_t run_heat;
    utils_ehsm_state_t run_clean;
    utils_ehsm_state_t run_probe_calibra;
    utils_ehsm_state_t run_bed_mesh;
    utils_ehsm_state_t error;
} auto_leveling_t;
static auto_leveling_t auto_leveling_ehsm = {0};
static int idle_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event);
static int run_heat_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event);
static int error_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event);
static int run_clean_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event);
static int run_probe_calibra_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event);
static int run_bed_mesh_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event);
static void auto_leveling_homing_state_callback(int state);
static void auto_leveling_probe_state_callback(probe_data_t state);
AutoLeveling::AutoLeveling(std::string section_name)
{
    bed_mesh_temp = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "bed_mesh_temp", DEFAULT_AUTO_LEVELING_BED_MESH_TEMP);                                       // 热床加热目标温度
    extruder_temp = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "extruder_temp", DEFAULT_AUTO_LEVELING_EXTRUDER_TEMP);                                       // 喷嘴加热目标温度
    move_speed = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "move_speed", DEFAULT_AUTO_LEVELING_MOVE_SPEED) * 60.0f;                                        // 移动速度(mm/s)
    extrude_length = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "extrude_length", DEFAULT_AUTO_LEVELING_EXTRUDE_LENGTH);                                    // 挤出料长度
    extrude_speed = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "extrude_speed", DEFAULT_AUTO_LEVELING_EXTRUDE_SPEED) * 60.0f;                               // 挤出料速度(mm/s)
    extruder_pullback_length = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "extruder_pullback_length", DEFAULT_AUTO_LEVELING_EXTRUDER_PULLBACK_LENGTH);      // 回抽料长度
    extruder_pullback_speed = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "extruder_pullback_speed", DEFAULT_AUTO_LEVELING_EXTRUDER_PULLBACK_SPEED) * 60.0f; // 回抽料速度(mm/s)
    extruder_cool_down_temp = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "extruder_cool_down_temp", DEFAULT_AUTO_LEVELING_EXTRUDER_COOL_DOWN_TEMP);         // 喷头冷却温度，冷却后取校准
    target_spot_x = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "target_spot_x", DEFAULT_AUTO_LEVELING_TARGET_SPOT_X);                                       // 校准模块X坐标，归零后的相对位置
    target_spot_y = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "target_spot_y", DEFAULT_AUTO_LEVELING_TARGET_SPOT_Y);                                       // 校准模块y坐标，归零后的相对位置
    extruder_wipe_z = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "extruder_wipe_z", DEFAULT_AUTO_LEVELING_EXTRUDER_WIPE_Z);                                 // 擦嘴的高度
    lifting_after_completion = Printer::GetInstance()->m_pconfig->GetDouble(section_name.c_str(), "lifting_after_completion", DEFAULT_AUTO_LEVELING_LIFTING_ATER_COMPLETION);       // 完成后抬升高度
    enable_fusion_trigger = Printer::GetInstance()->m_pconfig->GetBool(section_name.c_str(), "enable_fusion_trigger", DEFAULT_AUTO_LEVELING_ENABLE_FUSION_TRIGGER);          // 是能融合数据触发检测
    extrude_feed_gcode_str = Printer::GetInstance()->m_pconfig->GetString(section_name.c_str(), "extrude_feed_gcode", "");
    if(extrude_feed_gcode_str != "")
    {
        extrude_feed_gcode = split(extrude_feed_gcode_str, ",");
    }

    enable_test_mode = Printer::GetInstance()->m_pconfig->GetBool(section_name.c_str(), "enable_test_mode", false);
    Printer::GetInstance()->m_gcode->register_command("G29", std::bind(&AutoLeveling::cmd_G29, this, std::placeholders::_1));
    Printer::GetInstance()->m_gcode->register_command("G29.1", std::bind(&AutoLeveling::cmd_G29_1, this, std::placeholders::_1)); // 是否启用position和床网
    Printer::GetInstance()->m_gcode->register_command("WIPE_NOZZLE", std::bind(&AutoLeveling::cmd_WIPE_NOZZLE, this, std::placeholders::_1));
    Printer::GetInstance()->m_gcode->register_command("M729", std::bind(&AutoLeveling::cmd_M729, this, std::placeholders::_1));
    Printer::GetInstance()->m_gcode->register_command("CALIBRATE_Z_OFFSET", std::bind(&AutoLeveling::cmd_CALIBRATE_Z_OFFSET, this, std::placeholders::_1)); // 是否启用position和床网
    Printer::GetInstance()->m_gcode->register_command("LIFT_HOT_BED", std::bind(&AutoLeveling::cmd_LIFT_HOT_BED, this, std::placeholders::_1)); // 抬升热床
    // 初始化状态机
    utils_ehsm_create_state(&auto_leveling_ehsm.idle, NULL, idle_handler, "IDLE");
    utils_ehsm_create_state(&auto_leveling_ehsm.run_heat, NULL, run_heat_handler, "RUN_HEAT");
    utils_ehsm_create_state(&auto_leveling_ehsm.run_clean, NULL, run_clean_handler, "RUN_CLEAN");
    utils_ehsm_create_state(&auto_leveling_ehsm.run_probe_calibra, NULL, run_probe_calibra_handler, "PROBE_CALIBRA");
    utils_ehsm_create_state(&auto_leveling_ehsm.run_bed_mesh, NULL, run_bed_mesh_handler, "BED_MESH");
    utils_ehsm_create_state(&auto_leveling_ehsm.error, NULL, error_handler, "ERROR");
    utils_ehsm_create(&auto_leveling_ehsm.ehsm, "auto_leveling", &auto_leveling_ehsm.idle, NULL, AUTO_LEVELING_EVENT_QUEUE_SIZE, NULL);
    homing_register_state_callback(auto_leveling_homing_state_callback);
    probe_register_state_callback(auto_leveling_probe_state_callback);
    auto_levelling_doing = false;
    m_enable_fast_probe = false;
}
AutoLeveling::~AutoLeveling()
{
}
static int idle_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event)
{
    int ret = 1;
    int err;
    switch (event->event)
    {
    case UTILS_EHSM_IDLE_EVENT:
        ret = 0;
        break;
    case UTILS_EHSM_INIT_EVENT:
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_START: //---开始自动调平，Z轴先抬升---
        Printer::GetInstance()->m_gcode_io->single_command("M211 S0");
        Printer::GetInstance()->m_gcode_io->single_command("G29.1 P0");
        utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.run_heat);
        break;
    default:
        ret = 1;
        break;
    }
    return ret;
}
static int run_heat_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event)
{
    int ret = 1;
    int err;
    switch (event->event)
    {
    case UTILS_EHSM_IDLE_EVENT:
#if AUTO_LEVELING_EHSM_DEBUG == 0
        // if (Printer::GetInstance()->m_bed_heater->m_heater->m_smoothed_temp >= (Printer::GetInstance()->m_auto_leveling->bed_mesh_temp - 0.5f) && Printer::GetInstance()->m_printer_extruder->m_heater->m_smoothed_temp >= (Printer::GetInstance()->m_auto_leveling->extruder_temp - 0.5f))
        // {
        if (Printer::GetInstance()->m_printer_extruder->m_heater->m_smoothed_temp >= (Printer::GetInstance()->m_auto_leveling->extruder_temp - 0.5f))
        {
            utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_START_HOMING, NULL, 0); // 加热完成开始归零
        }
        else if (Printer::GetInstance()->m_printer_extruder->m_heater->m_smoothed_temp >= 50 && Printer::GetInstance()->m_printer_fan->get_status(0).speed < 0.1f) // 喷头加热到50度，开启模型散热风扇
        {
            Printer::GetInstance()->m_gcode_io->single_command("M106 S255"); //---开启散热风扇,模型散热风扇---
        }
#endif
        ret = 0;
        break;
    case UTILS_EHSM_INIT_EVENT:
#if AUTO_LEVELING_EHSM_DEBUG == 0
        Printer::GetInstance()->m_gcode_io->single_command("M190 S%.2f", Printer::GetInstance()->m_auto_leveling->bed_mesh_temp); //---热床升温,阻塞等待，目前先用着，有优化空间---
        Printer::GetInstance()->m_gcode_io->single_command("M104 S%.2f", Printer::GetInstance()->m_auto_leveling->extruder_temp); //---喷头加热---
#else
        utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_START_HOMING, NULL, 0);
#endif

        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_START_HOMING:
        if (Printer::GetInstance()->m_safe_z_homing != nullptr)
        {
            Printer::GetInstance()->m_safe_z_homing->m_home_x_pos = Printer::GetInstance()->m_pconfig->GetDouble("stepper_x", "position_endstop", 0.0f);
            Printer::GetInstance()->m_safe_z_homing->m_home_y_pos = Printer::GetInstance()->m_pconfig->GetDouble("stepper_y", "position_endstop", 0.0f);
        }
        Printer::GetInstance()->m_gcode_io->single_command("G28 X Y Z"); //---归零X Y Z，在X限位，Y限位处，归Z---
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_HOMING_Z_SUCC: //---归零成功---
        if (Printer::GetInstance()->m_safe_z_homing != nullptr)
        {
            Printer::GetInstance()->m_safe_z_homing->m_home_x_pos = Printer::GetInstance()->m_pconfig->GetDouble("safe_z_home", "home_x_position", 0.0f);
            Printer::GetInstance()->m_safe_z_homing->m_home_y_pos = Printer::GetInstance()->m_pconfig->GetDouble("safe_z_home", "home_y_position", 0.0f);
        }
        Printer::GetInstance()->m_gcode_io->single_command("M106 S255");
        Printer::GetInstance()->m_gcode_io->single_command("G0 X%2.f F%.2f", Printer::GetInstance()->m_auto_leveling->target_spot_x, Printer::GetInstance()->m_auto_leveling->move_speed); //---X移动到与靶点平行---
        utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.run_clean);
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_FALT: //---发生故障---
        if (Printer::GetInstance()->m_safe_z_homing != nullptr)
        {
            Printer::GetInstance()->m_safe_z_homing->m_home_x_pos = Printer::GetInstance()->m_pconfig->GetDouble("safe_z_home", "home_x_position", 0.0f);
            Printer::GetInstance()->m_safe_z_homing->m_home_y_pos = Printer::GetInstance()->m_pconfig->GetDouble("safe_z_home", "home_y_position", 0.0f);
        }
        utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.error);
        ret = 0;
        break;
    default:
        ret = 1;
        break;
    }
    return ret;
}
static int run_clean_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event)
{
    int ret = 1;
    int err;
    static bool z_homing_is_succ = false;
    switch (event->event)
    {
    case UTILS_EHSM_IDLE_EVENT:
#if AUTO_LEVELING_EHSM_DEBUG == 1
        if (z_homing_is_succ == true)
        {
            z_homing_is_succ = false;
            Printer::GetInstance()->m_gcode_io->single_command("G0 Z5.0 F%.2f", Printer::GetInstance()->m_auto_leveling->move_speed); // 擦嘴后抬升。
            utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.run_probe_calibra);
        }
#else
        if (Printer::GetInstance()->m_printer_extruder->m_heater->m_smoothed_temp <= Printer::GetInstance()->m_auto_leveling->extruder_cool_down_temp && z_homing_is_succ == true) // 等待降温并Z归零成功
        {
            z_homing_is_succ = false;
            Printer::GetInstance()->m_gcode_io->single_command("G0 Z5.0 F%.2f", Printer::GetInstance()->m_auto_leveling->move_speed); // 擦嘴后抬升。
            utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.run_probe_calibra);
        }
#endif
        ret = 0;
        break;
    case UTILS_EHSM_INIT_EVENT: //
#if AUTO_LEVELING_EHSM_DEBUG == 0
        Printer::GetInstance()->m_gcode_io->single_command("M83");
        Printer::GetInstance()->m_gcode_io->single_command("G0 E%.2f F%.2f I0", Printer::GetInstance()->m_auto_leveling->extrude_length, Printer::GetInstance()->m_auto_leveling->extrude_speed);                      //---喷嘴挤出，挤出长度和速度可配置---
        Printer::GetInstance()->m_gcode_io->single_command("G0 E-%.2f F%.2f I0", Printer::GetInstance()->m_auto_leveling->extruder_pullback_length, Printer::GetInstance()->m_auto_leveling->extruder_pullback_speed); //---回抽料，回抽长度和速度可配置---
#endif
        Printer::GetInstance()->m_gcode_io->single_command("G28 Z"); // 归零Z轴
        Printer::GetInstance()->m_gcode_io->single_command("M104 S0");
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_HOMING_Z_SUCC:
#if AUTO_LEVELING_EHSM_DEBUG == 0
        Printer::GetInstance()->m_gcode_io->single_command("M106 S255");
#endif
        Printer::GetInstance()->m_gcode_io->single_command("G90");                                                                                                                                                                                               //---开启散热风扇---                                                                                                                                                                                                                     //---归零成功---
        Printer::GetInstance()->m_gcode_io->single_command("G0 X%.2f Y%.2f F%.2f", Printer::GetInstance()->m_auto_leveling->target_spot_x + 25.0f, Printer::GetInstance()->m_auto_leveling->target_spot_y, Printer::GetInstance()->m_auto_leveling->move_speed); //---XY移动到擦嘴中心，速度可配置---
        Printer::GetInstance()->m_gcode_io->single_command("G0 Z%.2f F%.2f", Printer::GetInstance()->m_auto_leveling->extruder_wipe_z, Printer::GetInstance()->m_auto_leveling->move_speed);
        Printer::GetInstance()->m_gcode_io->single_command("G91"); // 下降Z轴，保证接触模块
        for (int i = 0; i < 3; i++)                                // 左右来回擦喷嘴
        {
            Printer::GetInstance()->m_gcode_io->single_command("G0 X-1.2 F300"); //------
            Printer::GetInstance()->m_gcode_io->single_command("G0 X2.4 F300");
            Printer::GetInstance()->m_gcode_io->single_command("G0 X-1.2 F300"); //------
        }
        z_homing_is_succ = true;
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_FALT: //---发生故障---
        utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.error);
        ret = 0;
        break;
    default:
        ret = 1;
        break;
    }
    return ret;
}
static int run_probe_calibra_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event)
{
    int ret = 1;
    int err;
    switch (event->event)
    {
    case UTILS_EHSM_IDLE_EVENT:
        ret = 0;
        break;
    case UTILS_EHSM_INIT_EVENT:
        Printer::GetInstance()->m_gcode_io->single_command("G90");
        Printer::GetInstance()->m_gcode_io->single_command("G0 X%.2f Y%.2f F%.2f", Printer::GetInstance()->m_auto_leveling->target_spot_x, Printer::GetInstance()->m_auto_leveling->target_spot_y, Printer::GetInstance()->m_auto_leveling->move_speed); //---XY移动校准模块，速度可配置---
        Printer::GetInstance()->m_gcode_io->single_command("PROBE_CALIBRATE");                                                                                                                                                                           //---校准---
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_PROBE_SUCC:
        utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.run_bed_mesh);
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_FALT:
        Printer::GetInstance()->m_gcode_io->single_command("G91");
        Printer::GetInstance()->m_gcode_io->single_command("G0 Z%.2f F%.2f", Printer::GetInstance()->m_auto_leveling->lifting_after_completion, Printer::GetInstance()->m_auto_leveling->move_speed); // 抬升                                                                                                //---发生故障---
        utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.error);
        ret = 0;
        break;
    default:
        ret = 1;
        break;
    }
    return ret;
}
static int run_bed_mesh_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event)
{
    int ret = 1;
    int err;
    switch (event->event)
    {
    case UTILS_EHSM_IDLE_EVENT:
        ret = 0;
        break;
    case UTILS_EHSM_INIT_EVENT:
        // Printer::GetInstance()->m_gcode_io->single_command("G28 X Y"); //---归零X Y Z---
        utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_HOMING_Y_SUCC, NULL, 0);
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_HOMING_Y_SUCC: //---归零X,Y成功---
        Printer::GetInstance()->m_gcode_io->single_command("M106 S255");
        Printer::GetInstance()->m_gcode_io->single_command("BED_MESH_CALIBRATE"); //---床网调平，生成网格数据---
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_FALT: //---发生故障---
        utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.error);
        ret = 0;
        break;
    case AUTO_LEVELING_EVENT_PROBE_SUCC:
        Printer::GetInstance()->m_gcode_io->single_command("G91");                                                                                                                                    //
        Printer::GetInstance()->m_gcode_io->single_command("G0 Z%.2f F%.2f", Printer::GetInstance()->m_auto_leveling->lifting_after_completion, Printer::GetInstance()->m_auto_leveling->move_speed); // 抬升
        Printer::GetInstance()->m_gcode_io->single_command("G28 X Y");
        Printer::GetInstance()->m_gcode_io->single_command("M140 S0"); //---关闭加热床---
        Printer::GetInstance()->m_gcode_io->single_command("M106 S0"); //---关闭散热风扇---
        Printer::GetInstance()->m_gcode_io->single_command("M211 S1");
        Printer::GetInstance()->m_gcode_io->single_command("G90");
        if(Printer::GetInstance()->m_strain_gauge->m_cfg->m_enable_z_home) {
            Printer::GetInstance()->m_gcode_io->single_command("G29.1 P1");
        }
        utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.idle);
        ret = 0;
        break;
    default:
        ret = 1;
        break;
    }
    return ret;
}
static int error_handler(utils_ehsm_t *ehsm, utils_ehsm_event_t *event)
{
    int ret = 1;
    int err;
    switch (event->event)
    {
    case UTILS_EHSM_IDLE_EVENT:
        ret = 0;
        break;
    case UTILS_EHSM_INIT_EVENT: // 可以在这里面添加回调
        Printer::GetInstance()->m_gcode_io->single_command("M211 S1");
        Printer::GetInstance()->m_gcode_io->single_command("M140 S0"); //---
        Printer::GetInstance()->m_gcode_io->single_command("M104 S0"); //------
        Printer::GetInstance()->m_gcode_io->single_command("M106 S0"); //---关闭散热风扇---
        Printer::GetInstance()->m_gcode_io->single_command("G90");
        if(Printer::GetInstance()->m_strain_gauge->m_cfg->m_enable_z_home) {
            Printer::GetInstance()->m_gcode_io->single_command("G29.1 P1");
        }
        utils_ehsm_tran(&auto_leveling_ehsm.ehsm, &auto_leveling_ehsm.idle);
        ret = 0;
        break;
    default:
        ret = 1;
        break;
    }
    return ret;
}
void auto_leveling_loop(void)
{
    utils_ehsm_run(&auto_leveling_ehsm.ehsm);
}
// void AutoLeveling::cmd_G29(GCodeCommand &gcmd)
// {
//     utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_START, NULL, 0);
// }

void AutoLeveling::cmd_LIFT_HOT_BED(GCodeCommand &gcmd)
{
    double distance = gcmd.get_double("S", 50., DBL_MIN, DBL_MAX, 0.,100.);
    double curtime = get_monotonic();
    LOG_I("lift hot bed:%f\n",distance);
    std::vector<double> pos = Printer::GetInstance()->m_tool_head->get_position();
    pos[2] = 0;
    std::vector<int> axes = {2};
    Printer::GetInstance()->m_tool_head->set_position(pos, axes);
    int step_cnt = (int)((distance) / (Printer::GetInstance()->m_strain_gauge->m_obj->m_dirzctl->m_steppers[0]->get_step_dist() * Printer::GetInstance()->m_strain_gauge->m_obj->m_dirzctl->m_step_base));
    int step_us = (int)((((distance) / Printer::GetInstance()->m_safe_z_homing->m_z_hop_speed) * 1000 * 1000) / step_cnt);
    Printer::GetInstance()->m_hx711s->CalibrationStart(50, false); 
    Printer::GetInstance()->m_hx711s->ProbeCheckTriggerStart(0, 0, 0); 
    Printer::GetInstance()->m_strain_gauge->m_obj->m_dirzctl->check_and_run(0, step_us, step_cnt, false, false);
    Printer::GetInstance()->m_strain_gauge->m_obj->m_hx711s->delay_s(0.015);
    std::cout << "PROBE_BY_STEP x= " << 0 << " y= " << 0 << " z= " << 0 << " speed_mm= " << Printer::GetInstance()->m_safe_z_homing->m_z_hop_speed << " step_us= " << step_us
            << " step_cnt= " << step_cnt << std::endl;
    std::cout << "******************************probe_by_step****************************" << std::endl;
    int32_t loop=0; 
    int32_t is_trigger=0;
    while (Printer::GetInstance()->m_strain_gauge->ck_sys_sta())
    {
        loop++;
        if (Printer::GetInstance()->m_strain_gauge->m_obj->m_dirzctl->m_params.size() == 2)
        {
            LOG_E("axis z wait lift timeout \n"); 
            break;
        }
        double running_dis = Printer::GetInstance()->m_safe_z_homing->m_z_hop_speed*loop/100.f;
        if (loop%100==0){
            std::cout << "running_dis: " << running_dis  << std::endl;
        }
        // if (running_dis > distance*2) {
        //     std::cout << "抬升结束。。。。。。。。running_dis:" << running_dis << " distance:"  << distance << std::endl;
        //     break;
        // }

        if (Printer::GetInstance()->m_hx711s->m_is_trigger > 0 && is_trigger==0 )
        {
            std::cout << "抬升中 probe_by_step trigger!!!" << std::endl;
            is_trigger = 1;
            loop = 0;
            running_dis = 0;
            // 反向 5mm 防止抬升时误触发，还在底部，
            distance = 5;
            Printer::GetInstance()->m_strain_gauge->m_obj->m_dirzctl->check_and_run(0, 0, 0, false);            //先停止，在反向
            usleep(10 * 1000);
            step_cnt = (int)(distance / (Printer::GetInstance()->m_strain_gauge->m_obj->m_dirzctl->m_steppers[0]->get_step_dist() * Printer::GetInstance()->m_strain_gauge->m_obj->m_dirzctl->m_step_base));
            step_us = (int)(((distance / Printer::GetInstance()->m_safe_z_homing->m_z_hop_speed) * 1000 * 1000) / step_cnt);
            Printer::GetInstance()->m_strain_gauge->m_obj->m_dirzctl->check_and_run(1, step_us, step_cnt);
        }
        usleep(10 * 1000);
    }
    Printer::GetInstance()->m_hx711s->ProbeCheckTriggerStop(true);
    Printer::GetInstance()->m_strain_gauge->m_obj->m_dirzctl->check_and_run(0, 0, 0, false);
    Printer::GetInstance()->m_tool_head->m_kin->note_z_not_homed();
}

void AutoLeveling::cmd_CALIBRATE_Z_OFFSET(GCodeCommand &gcmd)
{
    double old_position_endstop = Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop;
    Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop = 0;
    Printer::GetInstance()->m_gcode_io->single_command("G28 Z");
    Printer::GetInstance()->m_gcode_io->single_command("G1 Z5 F360");
    Printer::GetInstance()->m_gcode_io->single_command("PROBE");
    if (Printer::GetInstance()->m_probe->get_status(get_monotonic()).last_query)
    {
        double z_offset = Printer::GetInstance()->m_probe->get_status(get_monotonic()).last_z_result;
        // std::cout << "z_offset =" << z_offset << std::endl;
        LOG_I("z_offset = %.3f\n", z_offset);
        z_offset += 1.;
        Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop = -z_offset;
        Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop_extra = -z_offset;
        Printer::GetInstance()->m_pconfig->SetDouble("stepper_z", "position_endstop", Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop);
        Printer::GetInstance()->m_pconfig->SetDouble("stepper_z", "position_endstop_extra", Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop_extra);
        Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
        std::vector<string> keys;
        keys.push_back("position_endstop");
        keys.push_back("position_endstop_extra");
        Printer::GetInstance()->m_pconfig->WriteI_specified_Ini(USER_CONFIG_PATH, "stepper_z", keys);
        // std::cout << "m_position_endstop : " << Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop << std::endl;
        // std::cout << "m_position_endstop_extra : " << Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop_extra << std::endl;
    }
    else
    {
        LOG_E("CALIBRATE_Z_OFFSET ERROR\n");
    }
}

void AutoLeveling::cmd_WIPE_NOZZLE(GCodeCommand &gcmd)
{
    wipe_nozzle();
}

void AutoLeveling::cmd_M729(GCodeCommand &gcmd)
{
    std::vector<std::string> script;
    gcode_move_state_t gcode_status;
    gcode_status = Printer::GetInstance()->m_gcode_move->get_status(get_monotonic());
    bool absolute_coord = gcode_status.absolute_coord;
    script.push_back("G90");
    script.push_back("G1 X202 F6000");
    script.push_back("G1 Y264.5 F6000");
    script.push_back("G1 X173 F8000");
    for (uint8_t i = 0; i < 2; i++)
    {
        script.push_back("G1 X187");
        script.push_back("G1 X173");
    }
    script.push_back("G1 Y262.5 F800");
    script.push_back("G1 X187 F8000");
    for (uint8_t i = 0; i < 2; i++)
    {
        script.push_back("G1 X173");
        script.push_back("G1 X187");
    }
    script.push_back("G1 X173");
    script.push_back("G1 Y250");
    script.push_back("G1 X202");
    script.push_back("G1 Y264.5 F1200");
    script.push_back("M400");
    if(!absolute_coord)
        script.push_back("G91");

    Printer::GetInstance()->m_gcode->run_script(script);
}
void AutoLeveling::wipe_nozzle()
{

    ZMesh *mesh = Printer::GetInstance()->m_bed_mesh->get_mesh();
    Printer::GetInstance()->m_bed_mesh->set_mesh(nullptr);
    std::cout << "extrude_feed_gcode_str =" << extrude_feed_gcode_str << std::endl;
    // Printer::GetInstance()->m_gcode->run_script(extrude_feed_gcode);
    std::vector<std::string> script;
    script.push_back("G28");
    script.insert(script.end(), extrude_feed_gcode.begin(), extrude_feed_gcode.end());
    // script.push_back("G90");
    // script.push_back("G1 X202 F4500");
    // script.push_back("G1 Y266 F4500");
    // script.push_back("G1 Z0.5 F180");
    // script.push_back("M109 S250");
    // script.push_back("M83");
    // script.push_back("G1 E80 F600");
    // script.push_back("G1 E-30 F1800");
    // script.push_back("M82");
    // script.push_back("G91");
    // script.push_back("M106 S255");
    // for (int i = 0; i < 2; i++)
    // {
    //     script.push_back("G1 Y-10 F9000");
    //     script.push_back("G1 Y10 F9000");
    // }
    // script.push_back("M106 S0");
    // script.push_back("G1 Y-1.5 F1500");
    // script.push_back("G1 X-55 F1500");
    // // 擦喷嘴
    // for (int i = 0; i < 3; i++)
    // {
    //     script.push_back("G1 X35 F12000");
    //     script.push_back("G1 X-35 F12000");
    // }
    // script.push_back("G90");
    // script.push_back("G1 Z5 F360");
    // script.push_back("G1 X118 F4500");
    // script.push_back("G1 Y253 F4500");
    // // 堵住喷嘴 
    // script.push_back("STRAINGAUGE_Z_HOME");
    // script.push_back("M106 S255");
    // script.push_back("G91");
    // script.push_back("G1 Z-0.1 F360");
    // script.push_back("G1 X15 F120");
    // for(int i = 0; i < 3; i++)
    // {
    //     script.push_back("G1 X-8 F120");
    //     script.push_back("G1 X8 F120");
    // }
    // script.push_back("G90");
    // script.push_back("M109 S140");
    // script.push_back("M106 S0");
    Printer::GetInstance()->m_gcode->run_script(script);
    Printer::GetInstance()->m_bed_mesh->set_mesh(mesh);
}

void AutoLeveling::cmd_G29(GCodeCommand &gcmd)
{
    std::string profile_name = gcmd.get_string("PROFILE", "");
    if (profile_name.empty())
    {
        profile_name = gcmd.get_string("SAVE", "");
    }
    std::string probe_count = gcmd.get_string("PROBE_COUNT", "");

    auto_leveling_state_callback_call(AUTO_LEVELING_STATE_START);
    if(!enable_test_mode){
        // Printer::GetInstance()->m_gcode_io->single_command("G28");
        if(Printer::GetInstance()->m_strain_gauge->m_cfg->m_enable_z_home) {
            Printer::GetInstance()->m_gcode_io->single_command("G29.1 P0");
        }
        Printer::GetInstance()->m_gcode_io->single_command("M211 S0");  //软限位
        auto_leveling_state_callback_call(AUTO_LEVELING_STATE_START_PREHEAT);
        Printer::GetInstance()->m_gcode_io->single_command("M104 S140");
        Printer::GetInstance()->m_gcode_io->single_command("M140 S60");
        Printer::GetInstance()->m_gcode_io->single_command("M106 P1 S0");
        Printer::GetInstance()->m_gcode_io->single_command("M106 P2 S0");
        Printer::GetInstance()->m_gcode_io->single_command("M106 P3 S0"); 
        wipe_nozzle();
        Printer::GetInstance()->m_gcode_io->single_command("M140 S60");  //开启热床加热：60
        Printer::GetInstance()->m_gcode_io->single_command("RESET_PRINTER_PARAM");
        Printer::GetInstance()->m_gcode_io->single_command("M109 S140");    //阻塞加热
        // Printer::GetInstance()->m_gcode_io->single_command("M190 S60");
        // Printer::GetInstance()->m_gcode_io->single_command("G1 Z20 F900");
        // auto_leveling_state_callback_call(AUTO_LEVELING_STATE_START_EXTURDE);
        // Printer::GetInstance()->m_gcode_io->single_command("G1 E50 F180");
        // Printer::GetInstance()->m_gcode_io->single_command("G1 -E35 F1800");
        // Printer::GetInstance()->m_gcode_io->single_command("M104 S0");
        // Printer::GetInstance()->m_tool_head->wait_moves();
    }

    auto_levelling_doing = true;
    // Printer::GetInstance()->m_gcode_io->single_command("CALIBRATE_Z_OFFSET");
    // if (!Printer::GetInstance()->m_probe->get_status(get_monotonic()).last_query)
    // {
    //     Printer::GetInstance()->m_gcode_io->single_command("G28 X Y");
    //     return;
    // }
    Printer::GetInstance()->m_gcode_io->single_command("SET_GCODE_OFFSET Z=0 MOVE=0 MOVE_SPEED=5.0");
    Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop = Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop_extra;
    Printer::GetInstance()->m_gcode_io->single_command("M8233 S%.3f P1", 0);
    // Printer::GetInstance()->m_probe->m_z_offset = 0;
    // Printer::GetInstance()->m_bed_mesh_probe->m_z_offset = 0;
    Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);

    // printf("m_rails[2]->m_position_endstop:%f m_rails[2]->m_position_endstop_extra:%f\n", Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop,Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop_extra);
    // double position_endstop_extra = Printer::GetInstance()->m_pconfig->GetDouble("stepper_z", "position_endstop_extra", 4.5);
    // double position_endstop = Printer::GetInstance()->m_pconfig->GetDouble("stepper_z", "position_endstop", 4.5);
    // printf("position_endstop:%f position_endstop_extra:% f", position_endstop,position_endstop_extra);
    // Printer::GetInstance()->m_pconfig->SetDouble("stepper_z", "position_endstop", position_endstop_extra);
    // Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
    // Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop = Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop_extra;
    
    Printer::GetInstance()->m_gcode_io->single_command("G28 Z");


    // Printer::GetInstance()->m_gcode_io->single_command("G90");
    // for () {
    //     Printer::GetInstance()->m_gcode_io->single_command("G1 X120 Y120 Z5 F6000"); 
    //     Printer::GetInstance()->m_gcode_io->single_command("PROBE_ACCURACY PROBE_SPEED=3 SAMPLES=50 SAMPLE_RETRACT_DIST=3"); //
    // }

    // Printer::GetInstance()->m_gcode_io->single_command("G1 X%.2f Y%.2f Z%.2f F600", now_pos[0], now_pos[1], now_pos[2] - m_cfg->m_shake_range / 2);
    
    Printer::GetInstance()->m_gcode_io->single_command("M109 S140");
    Printer::GetInstance()->m_gcode_io->single_command("M190 S60");
    // Printer::GetInstance()->m_gcode_io->single_command("M104 S0");     //关闭喷头加热
    // Printer::GetInstance()->m_gcode_io->single_command("M140 S0");     //关闭热床加热

    auto_leveling_state_callback_call(AUTO_LEVELING_STATE_START_PROBE);
    try 
    {
        if (m_enable_fast_probe) {
            std::cout << "G29 BED_MESH_CALIBRATE fast" << std::endl;
            std::map<std::string, std::string> params;
            params["METHOD"] = "fast";
            if (!profile_name.empty())
            {
                params["PROFILE"] = profile_name;
            }
            if (!probe_count.empty())
            {
                params["PROBE_COUNT"] = probe_count;
            }
            GCodeCommand bed_mesh_calibrate_cmd = Printer::GetInstance()->m_gcode->create_gcode_command("BED_MESH_CALIBRATE", "BED_MESH_CALIBRATE", params);
            Printer::GetInstance()->m_bed_mesh->m_bmc->cmd_BED_MESH_CALIBRATE(bed_mesh_calibrate_cmd);
            // Printer::GetInstance()->m_gcode_io->single_command("BED_MESH_CALIBRATE METHOD=fast"); //
        } else {
            std::cout << "G29 BED_MESH_CALIBRATE normal" << std::endl;
            std::map<std::string, std::string> params;
            if (!profile_name.empty())
            {
                params["PROFILE"] = profile_name;
            }
            if (!probe_count.empty())
            {
                params["PROBE_COUNT"] = probe_count;
            }
            GCodeCommand bed_mesh_calibrate_cmd = Printer::GetInstance()->m_gcode->create_gcode_command("BED_MESH_CALIBRATE", "BED_MESH_CALIBRATE", params);
            Printer::GetInstance()->m_bed_mesh->m_bmc->cmd_BED_MESH_CALIBRATE(bed_mesh_calibrate_cmd);
            // Printer::GetInstance()->m_gcode_io->single_command("BED_MESH_CALIBRATE "); //
        }
        auto_levelling_doing = false;
    } 
    catch (...)
    {
        LOG_E("BED_MESH_CALIBRATE error\n");
        throw;
    }
    auto_leveling_state_callback_call(AUTO_LEVELING_STATE_FINISH);
    Printer::GetInstance()->m_gcode_io->single_command("G90");
    Printer::GetInstance()->m_gcode_io->single_command("G1 X10 Y10 F4500");
    Printer::GetInstance()->m_gcode_io->single_command("M106 S0");
    Printer::GetInstance()->m_gcode_io->single_command("M104 S0");
    Printer::GetInstance()->m_gcode_io->single_command("M140 S0");

    if(Printer::GetInstance()->m_strain_gauge->m_cfg->m_enable_z_home) {
        Printer::GetInstance()->m_gcode_io->single_command("G29.1 P1");
    }
    Printer::GetInstance()->m_gcode_io->single_command("M211 S1");
    auto_leveling_state_callback_call(AUTO_LEVELING_STATE_RESET);
}
/**
 * @brief
 *
 * @param state
 */
static void auto_leveling_homing_state_callback(int state)
{
    switch (state)
    {
    case HOMING_STATE_X_FALT:
    case HOMING_STATE_Y_FALT:
    case HOMING_STATE_Z_FALT:
        utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_FALT, NULL, 0);
        break;
    case HOMING_STATE_Z_SUCC:
        utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_HOMING_Z_SUCC, NULL, 0);
        break;
    case HOMING_STATE_Y_SUCC:
        utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_HOMING_Y_SUCC, NULL, 0);
        break;
    default:
        break;
    }
}
static void auto_leveling_probe_state_callback(probe_data_t state)
{
    switch (state.state)
    {
    case CMD_PROBE_FALT:
        utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_FALT, NULL, 0);
        break;
    case CMD_PROBE_SUCC:
        utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_PROBE_SUCC, NULL, 0);
        break;
    case CMD_BEDMESH_PROBE_FALT:
    case CMD_BEDMESH_PROBE_EXCEPTION:
        utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_FALT, NULL, 0);
        break;
    case CMD_BEDMESH_PROBE_SUCC:
        utils_ehsm_event(&auto_leveling_ehsm.ehsm, AUTO_LEVELING_EVENT_PROBE_SUCC, NULL, 0);
        break;
    default:
        break;
    }
}
/**
 * @brief 修改是否应用z_offset(position_endstop),和床网。不支持多线程调用！注意传入参数，不要传0或1。
 *
 * @param
 */
void AutoLeveling::cmd_G29_1(GCodeCommand &gcmd)
{
    if (Printer::GetInstance()->m_bed_mesh == nullptr || Printer::GetInstance()->m_probe == nullptr || Printer::GetInstance()->m_bed_mesh_probe == nullptr)
    {
        return;
    }
    int is_apply = gcmd.get_int("P", 1);
    if (is_apply == 1)
    {
        std::cout << "----------------------------------------G29 P1--------------------------------------------------------" << std::endl;
        Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop = Printer::GetInstance()->m_pconfig->GetDouble("stepper_z", "position_endstop", DEFAULT_STEPPER_Z_POSITION_ENDSTOP); //---Z轴归零后的坐标---
        Printer::GetInstance()->m_probe->m_z_offset = Printer::GetInstance()->m_pconfig->GetDouble("probe", "z_offset", DEFAULT_PROBE_Z_OFFSET);                                                        //---这个值未必有应用，目前主要用m_position_endstop---
        Printer::GetInstance()->m_bed_mesh_probe->m_z_offset = Printer::GetInstance()->m_pconfig->GetDouble("probe", "z_offset", DEFAULT_PROBE_Z_OFFSET);                                               //---这个值未必有应用，目前主要用m_position_endstop---
        Printer::GetInstance()->m_bed_mesh->m_pmgr->load_profile(Printer::GetInstance()->m_bed_mesh->m_pmgr->get_current_profile());
    }
    else
    {
        std::cout << "----------------------------------------G29 P0--------------------------------------------------------" << std::endl;
        Printer::GetInstance()->m_gcode_move->m_base_position[2] = 0;
        Printer::GetInstance()->m_gcode_move->m_homing_position[2] = 0;
        Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop = 0.0f; //---Z轴归零后的坐标---
        Printer::GetInstance()->m_probe->m_z_offset = 0.0f;                                //---这个值未必有应用，目前主要用m_position_endstop---
        Printer::GetInstance()->m_bed_mesh_probe->m_z_offset = 0.0f;                       //---这个值未必有应用，目前主要用m_position_endstop---
        Printer::GetInstance()->m_bed_mesh->set_mesh(NULL);                                //---清除网格应用---
    }
}
const char *get_auto_level_cur_state_name(void)
{
    if (auto_leveling_ehsm.ehsm.current_state == NULL)
    {
        return NULL;
    }
    return auto_leveling_ehsm.ehsm.current_state->name;
}
const char *get_auto_level_last_state_name(void)
{
    if (auto_leveling_ehsm.ehsm.last_state == NULL)
    {
        return NULL;
    }
    return auto_leveling_ehsm.ehsm.last_state->name;
}

#define AUTO_LEVELING_STATE_CALLBACK_SIZE 16
static auto_leveling_state_callback_t auto_leveling_state_callback[AUTO_LEVELING_STATE_CALLBACK_SIZE];

int auto_leveling_register_state_callback(auto_leveling_state_callback_t state_callback)
{
    for (int i = 0; i < AUTO_LEVELING_STATE_CALLBACK_SIZE; i++)
    {
        if (auto_leveling_state_callback[i] == NULL)
        {
            auto_leveling_state_callback[i] = state_callback;
            return 0;
        }
    }
    return -1;
}

int auto_leveling_unregister_state_callback(auto_leveling_state_callback_t state_callback)
{
    for (int i = 0; i < AUTO_LEVELING_STATE_CALLBACK_SIZE; i++)
    {
        if (auto_leveling_state_callback[i] == state_callback)
        {
            auto_leveling_state_callback[i] = NULL;
            return 0;
        }
    }
    return -1;
}

int auto_leveling_state_callback_call(int state)
{
    for (int i = 0; i < AUTO_LEVELING_STATE_CALLBACK_SIZE; i++)
    {
        if (auto_leveling_state_callback[i] != NULL)
        {
            auto_leveling_state_callback[i](state);
        }
    }
    return 0;
}
