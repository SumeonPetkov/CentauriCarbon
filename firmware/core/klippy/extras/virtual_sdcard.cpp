#include "virtual_sdcard.h"
#include "klippy.h"
#include "Define.h"
#include "my_string.h"
#include "print_stats.h"
#include "Define_config_path.h"
#include <cctype>
#define LOG_TAG "sdcard"
#undef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#include "log.h"
#include "hl_common.h"
// VALID_GCODE_EXTS = ["gcode", "g", "gco"]

#define PROFILE_PREFIX_SUFFIX_NAME "default"
#define ACTIVE_MESH_PROFILE_OPTION "active_mesh_profile"

static std::string normalize_mesh_profile_name(const std::string &name)
{
    std::string normalized;
    normalized.reserve(name.size());
    for (char c : name)
    {
        unsigned char uc = static_cast<unsigned char>(c);
        if (isalnum(uc) || c == '_')
        {
            normalized.push_back((char)tolower(uc));
        }
        else if (c == '-' || c == ' ')
        {
            normalized.push_back('_');
        }
    }
    return normalized;
}

VirtualSD::VirtualSD(std::string section_name)
{
    Printer::GetInstance()->register_event_handler("klippy:shutdown:" + section_name, std::bind(&VirtualSD::handle_shutdown, this));
    // sdcard state
    m_sdcard_dirname = Printer::GetInstance()->m_pconfig->GetString(section_name, "path");
    m_sdcard_dirname = USB_DISK_PATH;
    m_sdcard_dirname += "/";

    m_valid_gcode_exts = {"gcode", "g", "gco"};
    m_file_position = 0;
    m_file_size = 0;
    m_cancel = false;
    // Print Stat Tracking
    // Work timer
    m_must_pause_work = false;
    m_cmd_from_sd = false;
    m_next_file_position = 0;
    m_is_active = false;
    // Register commands
    // Printer::GetInstance()->m_gcode->register_command("M20", std::bind(&VirtualSD::cmd_M20, this, std::placeholders::_1));
    // Printer::GetInstance()->m_gcode->register_command("M21", std::bind(&VirtualSD::cmd_M21, this, std::placeholders::_1));
    // Printer::GetInstance()->m_gcode->register_command("M23", std::bind(&VirtualSD::cmd_M23, this, std::placeholders::_1));
    // Printer::GetInstance()->m_gcode->register_command("M24", std::bind(&VirtualSD::cmd_M24, this, std::placeholders::_1));
    // Printer::GetInstance()->m_gcode->register_command("M25", std::bind(&VirtualSD::cmd_M25, this, std::placeholders::_1));
    // Printer::GetInstance()->m_gcode->register_command("M26", std::bind(&VirtualSD::cmd_M26, this, std::placeholders::_1));
    // Printer::GetInstance()->m_gcode->register_command("M27", std::bind(&VirtualSD::cmd_M27, this, std::placeholders::_1));
    // Printer::GetInstance()->m_gcode->register_command("M28", std::bind(&VirtualSD::cmd_M28, this, std::placeholders::_1));
    // Printer::GetInstance()->m_gcode->register_command("M29", std::bind(&VirtualSD::cmd_M29, this, std::placeholders::_1));
    // Printer::GetInstance()->m_gcode->register_command("M30", std::bind(&VirtualSD::cmd_M30, this, std::placeholders::_1));

    m_cmd_SDCARD_RESET_FILE_help = ""; // "Clears a loaded SD File. Stops the print if necessary"
    m_cmd_SDCARD_PRINT_FILE_help = ""; // "Loads a SD file and starts the print.  May include files in subdirectories."
    Printer::GetInstance()->m_gcode->register_command("SDCARD_RESET_FILE", std::bind(&VirtualSD::cmd_SDCARD_RESET_FILE, this, std::placeholders::_1), false, m_cmd_SDCARD_RESET_FILE_help);
    Printer::GetInstance()->m_gcode->register_command("SDCARD_PRINT_FILE", std::bind(&VirtualSD::cmd_SDCARD_PRINT_FILE, this, std::placeholders::_1), false, m_cmd_SDCARD_PRINT_FILE_help);
    Printer::GetInstance()->m_gcode->register_command("SDCARD_PRINT_FILE_BREAK_SAVE", std::bind(&VirtualSD::cmd_SDCARD_PRINT_FILE_from_break, this, std::placeholders::_1), false, m_cmd_SDCARD_PRINT_FILE_help);
}

VirtualSD::~VirtualSD()
{
}

void VirtualSD::handle_shutdown()
{
    // if (is_work_active())
    // {
        m_must_pause_work = true;
        int readpos = std::max(m_file_position - 1024, 0);
        int readcount = m_file_position - readpos;
        m_current_file.seekg(0, std::ios::beg);
        char buf[readcount + 128];
        m_current_file.read(buf, readcount + 128);

        // logging.info("Virtual sdcard (%d): %s\nUpcoming (%d): %s",
        //                 readpos, repr(data[:readcount]),
        //                 m_file_position, repr(data[readcount:]))
    // }
}

std::string VirtualSD::stats(double eventtime)
{
    // if (m_work_timer == nullptr)
    //     return False, ""
    // return True, "sd_pos=%d" % (m_file_position,)
}

std::vector<file_info_t> VirtualSD::get_file_list(bool check_subdirs)
{
#if 0
    std::map<std::string, int> flist;
    if (check_subdirs)
    {
        DIR *dir = opendir(m_sdcard_dirname.c_str());
        if (dir == NULL) {
            printf("open file fail!\n");
            return flist;
        }
        struct dirent *ptr;
        while ((ptr = readdir(dir)) != NULL)
        {
            std::cout << "ptr->d_name = " << ptr->d_name << std::endl;
            if(strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            {
                continue;
            }

            if (ptr->d_type == 1) //文件
            {
                
                if (strstr(ptr->d_name, ".") != NULL) //判断有无后缀
                {
                    if (strcmp(strstr(ptr->d_name, "."), ".NC") == 0 || strcmp(strstr(ptr->d_name, "."), ".nc") == 0 || strcmp(strstr(ptr->d_name, "."), ".gc") == 0 || strcmp(strstr(ptr->d_name, "."), ".GC") == 0) //判断是否符合后缀
                    {
                        if (strlen(ptr->d_name) > 255) { //文件名过长，过滤掉
                            continue;
                        }
                        flist[ptr->d_name] = ptr->d_reclen;
                    }
                }
                
            }

        return flist;
        }
        closedir(dir);
    }
#endif
    std::vector<file_info_t> file_infos;
    LOG_I("m_sdcard_dirname = %s\n", m_sdcard_dirname.c_str());
    DIR *dir = opendir(m_sdcard_dirname.c_str());
    if (dir == NULL)
    {
        LOG_I("open file fail!\n");
        return file_infos;
    }
    struct dirent *ptr;
    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0 || strcmp(ptr->d_name, "System Volume Information") == 0)
        {
            continue;
        }
        LOG_I("ptr->d_name = %s\n", ptr->d_name);
        // std::cout << "ptr->d_name = " << ptr->d_name << std::endl;
        file_info_t file_info;
        struct stat info;
        stat(ptr->d_name, &info);
        file_info.name = ptr->d_name;
        file_info.info = info;
        file_infos.push_back(file_info);
    }
    closedir(dir);
    return file_infos;
}

virtual_sd_stats_t VirtualSD::get_status(double eventtime)
{
    virtual_sd_stats_t ret = {
        .file_path = file_path(),
        .progress = progress(),
        .is_active = is_active(),
        .file_position = m_file_position,
        .file_size = m_file_size,
    };
    return ret;
}

std::string VirtualSD::file_path()
{
    if (m_current_file.is_open())
    {
        return m_current_file_name;
    }
    return "";
}

double VirtualSD::progress()
{
    if (m_file_size != 0)
        return (double)(m_file_position / m_file_size);
    else
        return 0.;
}

bool VirtualSD::is_active()
{
    return m_is_active;
}

bool VirtualSD::start_print_from_sd()
{
    return m_start_print_from_sd;
}

bool VirtualSD::get_cancel_flag()
{
    return m_cancel;
}

bool VirtualSD::is_printing()
{
    return m_current_file.is_open() && is_work_active();
}

void VirtualSD::resume_active()
{
    m_is_active = true;
}

void VirtualSD::do_pause()
{
    if (is_work_active())
    {
        // 如果当前在work_handler中，只设置标记
        if (m_in_work_handler)
        {
            m_must_pause_work = true;
            m_is_active = false;
            return;
        }

        // 否则直接注销定时器
        Printer::GetInstance()->m_reactor->unregister_timer(m_work_timer);
    }
    m_must_pause_work = true;
    m_is_active = false;
    m_start_print_from_sd = false;
    Printer::GetInstance()->m_gcode_io->single_command("SAVE_VARIABLE VARIABLE=virtual_sd_stats-m_is_active VALUE=" + to_string(m_is_active));
}

void VirtualSD::do_resume()
{
    if (!m_current_file.is_open())
    {
        return;
    }

    m_must_pause_work = false;
    m_is_active = true;
    m_start_print_from_sd = true;

    if (!is_work_active())
    {
        m_work_timer = Printer::GetInstance()->m_reactor->register_timer(
            "sd_work",
            std::bind(&VirtualSD::work_handler, this, std::placeholders::_1),
            Printer::GetInstance()->m_reactor->m_NOW);
    }
}

void VirtualSD::do_cancel()
{
    m_must_pause_work = true;
    m_is_active = false;
    m_cancel = true;
    
    do_pause();
    if (m_current_file.is_open())
    {
        m_current_file.close();
    }
    
    // 清理所有缓存和状态
    m_file_position = m_file_size = 0;
    m_next_file_position = 0;
    m_partial_input.clear();
    m_lines.clear();
    m_pending_command.clear();  // 清理切料相关的命令缓存
    m_start_print_from_sd = false;
    custom_zero = false;
    m_cmd_from_sd = false;
    
    // 确保文件完全关闭和重置
    m_current_file_name.clear();
    
    Printer::GetInstance()->m_gcode_io->single_command("SAVE_VARIABLE VARIABLE=virtual_sd_stats-m_is_active VALUE=" + to_string(m_is_active));
}

// G-Code commands
void VirtualSD::cmd_error(GCodeCommand &gcmd)
{
    // raise gcmd.error("SD write not supported")
}

void VirtualSD::_reset_file()
{
    if (m_current_file.is_open())
    {
        do_pause();
        m_current_file.close();
    }
    
    // 清理文件相关的所有状态
    m_file_position = m_file_size = 0;
    m_next_file_position = 0;
    m_partial_input.clear();
    m_lines.clear();
    m_current_file_name.clear();
    
    Printer::GetInstance()->m_print_stats->reset();
}

void VirtualSD::cmd_SDCARD_RESET_FILE(GCodeCommand &gcmd)
{
    if (m_cmd_from_sd)
    {
        // raise gcmd.error("SDCARD_RESET_FILE cannot be run from the sdcard")
    }
    _reset_file();
}

void VirtualSD::cmd_SDCARD_PRINT_FILE(GCodeCommand &gcmd)
{
    if (is_work_active())
    {
        LOG_E("SD busy\n");
        return;
        // raise m_gcode.error("SD busy")
    }

    std::string filename = gcmd.get_string("FILENAME", "");
    m_file_slicer = gcmd.get_int("FILESLICE", 0);
    // std::cout << "cmd_SDCARD_PRINT_FILE m_file_slicer = " << m_file_slicer << std::endl;
    _reset_file();
    LOG_I("print filename = %s\n", filename.c_str());
    // std::cout << "filename = " << filename << std::endl;
    // if (filename[0] == '/')
    // {
    //     filename = filename.substr(1);
    // }
    _load_file(gcmd, filename, true);
    if (Printer::GetInstance()->m_bed_mesh == nullptr || Printer::GetInstance()->m_bed_mesh->m_pmgr == nullptr)
    {
        LOG_E("Print aborted: bed mesh subsystem is unavailable\n");
        gcmd.m_respond_info("Print aborted: bed mesh subsystem is unavailable", true);
        _reset_file();
        return;
    }
    std::string explicit_mesh = gcmd.get_string("MESH", "");
    std::string normalized_explicit_mesh = normalize_mesh_profile_name(explicit_mesh);
    std::string active_mesh = normalize_mesh_profile_name(Printer::GetInstance()->m_pconfig->GetString("bed_mesh", ACTIVE_MESH_PROFILE_OPTION, ""));
    std::string selected_mesh = "";
    std::string selected_source = "";
    bool mesh_loaded = false;

    if (normalized_explicit_mesh != "")
    {
        selected_mesh = normalized_explicit_mesh;
        selected_source = "explicit";
        mesh_loaded = Printer::GetInstance()->m_bed_mesh->m_pmgr->load_profile_by_name(selected_mesh);
    }
    else if (active_mesh != "")
    {
        selected_mesh = active_mesh;
        selected_source = "active";
        mesh_loaded = Printer::GetInstance()->m_bed_mesh->m_pmgr->load_profile_by_name(selected_mesh);
    }
    else
    {
        selected_mesh = PROFILE_PREFIX_SUFFIX_NAME;
        selected_source = "fallback";
        mesh_loaded = Printer::GetInstance()->m_bed_mesh->m_pmgr->load_profile(PROFILE_PREFIX_SUFFIX_NAME);
    }
    if (!mesh_loaded)
    {
        LOG_E("Unable to resolve bed mesh profile from %s source (%s)\n", selected_source.c_str(), selected_mesh.c_str());
        gcmd.m_respond_info("Print aborted: missing bed mesh profile [" + selected_mesh + "] from " + selected_source + " source", true);
        _reset_file();
        return;
    }
    Printer::GetInstance()->m_bed_mesh->set_active_mesh_profile(selected_mesh);
    LOG_I("Using bed mesh profile [%s] from %s source\n", selected_mesh.c_str(), selected_source.c_str());

    if (Printer::GetInstance()->m_break_save != nullptr)
    {
        Printer::GetInstance()->m_break_save->load_variables();
        if (Printer::GetInstance()->m_break_save->m_virtual_sd_stats_Variables.find("file_path") != Printer::GetInstance()->m_break_save->m_virtual_sd_stats_Variables.end())
        {
            if (Printer::GetInstance()->m_break_save->m_virtual_sd_stats_Variables["file_path"] == filename)
            {
                Printer::GetInstance()->m_break_save->m_break_save_reserve = true;
                std::cout << std::endl;
                std::cout << "cmd_SDCARD_PRINT_FILE_from_break :" << filename << std::endl;
                std::cout << std::endl;
                cmd_SDCARD_PRINT_FILE_from_break(gcmd);
            }
        }
        Printer::GetInstance()->m_break_save->delete_save_files();
    }
    Printer::GetInstance()->m_break_save->m_break_save_reserve = false;
    do_resume();
}

void VirtualSD::cmd_SDCARD_PRINT_FILE_from_break(GCodeCommand &gcmd)
{
    static int printing_speed_value = 100; // 打印速度值
    if (!m_current_file.is_open())
    {
        return;
    }
    if (Printer::GetInstance()->m_break_save->m_virtual_sd_stats_Variables.find("file_position") != Printer::GetInstance()->m_break_save->m_virtual_sd_stats_Variables.end())
    {
        if (Printer::GetInstance()->m_break_save->s_saved_print_para.offset_size)
        {
            // Printer::GetInstance()->m_gcode_io->single_command("SET_PRINT_STATS_INFO CURRENT_LAYER=%d", Printer::GetInstance()->m_break_save->s_saved_print_para.current_layer);
            Printer::GetInstance()->m_gcode_io->single_command("SET_PRINT_STATS_INFO CURRENT_LAYER=%d TOTAL_LAYERS=%d", Printer::GetInstance()->m_break_save->s_saved_print_para.curr_layer_num, Printer::GetInstance()->m_break_save->s_saved_print_para.total_layer_num);
            Printer::GetInstance()->m_gcode_io->single_command("SET_PRINT_STATS_INFO LAST_PRINT_TIME=" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.last_print_time));
            // Printer::GetInstance()->m_print_stats->m_print_stats.last_print_time = Printer::GetInstance()->m_break_save->s_saved_print_para.last_print_time;
            Printer::GetInstance()->m_print_stats->m_print_stats.filament_used = Printer::GetInstance()->m_break_save->s_saved_print_para.filament_used;
            std::cout << "SET_PRINT_STATS_INFO ALREAD_PRINT_TIME ALREAD_PRINT_TIME=" << Printer::GetInstance()->m_break_save->s_saved_print_para.alread_print_time << std::endl;
            m_file_position = Printer::GetInstance()->m_break_save->s_saved_print_para.offset_size;
            m_current_file.seekg(m_file_position, std::ios::beg);
            std::vector<double> pos = {Printer::GetInstance()->m_break_save->s_saved_print_para.save_x_stop, Printer::GetInstance()->m_break_save->s_saved_print_para.save_y_stop, Printer::GetInstance()->m_break_save->s_saved_print_para.save_z_stop};
            Printer::GetInstance()->m_tool_head->set_position(pos, {2});
            // Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_min = Printer::GetInstance()->m_break_save->s_saved_print_para.m_limits_z[0];
            // Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_max = Printer::GetInstance()->m_break_save->s_saved_print_para.m_limits_z[1];
            Printer::GetInstance()->m_gcode_io->single_command("M140 S" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.bed_temp));
            Printer::GetInstance()->m_gcode_io->single_command("M104 S" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.end_temp[0]));

            Printer::GetInstance()->m_gcode_io->single_command("M190 S" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.bed_temp));
            Printer::GetInstance()->m_gcode_io->single_command("M109 S" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.end_temp[0]));
            Printer::GetInstance()->m_gcode_io->single_command("M106 P1 S255");
            Printer::GetInstance()->m_gcode_io->single_command("M106 P2 S" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.m_model_helper_fan_speed));
            Printer::GetInstance()->m_gcode_io->single_command("M106 P3 S" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.m_box_fan_speed));

            Printer::GetInstance()->m_gcode_io->single_command("STOP_RESUME X=" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.X) + " Y=" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.Y) + " Z=" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.Z) + " E=" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.E[0]) + " F=" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.speed) + " save_z_stop=" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.save_z_stop));
            // Printer::GetInstance()->m_tool_head->set_position(pos, {2});
            // Printer::GetInstance()->m_gcode_io->single_command("G92 E" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.E[0]));
            Printer::GetInstance()->m_gcode_io->single_command("M106 S" + to_string(Printer::GetInstance()->m_break_save->s_saved_print_para.m_printer_fan_speed));
            if (Printer::GetInstance()->m_break_save->s_saved_print_para.absolute_coord)
            {
                Printer::GetInstance()->m_gcode_io->single_command("G90");
            }
            else
            {
                Printer::GetInstance()->m_gcode_io->single_command("G91");
            }
            if (Printer::GetInstance()->m_break_save->s_saved_print_para.absolute_extrude)
            {
                Printer::GetInstance()->m_gcode_io->single_command("M82");
            }
            else
            {
                Printer::GetInstance()->m_gcode_io->single_command("M83");
            }
            Printer::GetInstance()->m_gcode_io->single_command("SET_PRINT_STATS_INFO CURRENT_LAYER=%d TOTAL_LAYERS=%d", Printer::GetInstance()->m_break_save->s_saved_print_para.curr_layer_num, Printer::GetInstance()->m_break_save->s_saved_print_para.total_layer_num);
            Printer::GetInstance()->m_print_stats->m_print_stats.last_print_time = Printer::GetInstance()->m_break_save->s_saved_print_para.last_print_time;
            Printer::GetInstance()->m_print_stats->m_print_stats.filament_used = Printer::GetInstance()->m_break_save->s_saved_print_para.filament_used;
            Printer::GetInstance()->m_break_save->m_break_save_files_status = true;
            printing_speed_value = 50;
            ui_cb[set_print_speed_cb](&printing_speed_value);
        }
    }
}

void VirtualSD::cmd_M20(GCodeCommand &gcmd)
{
    // List SD card
    std::vector<file_info_t> files = get_file_list();
    gcmd.m_respond_raw("Begin file list\r\n");
    for (auto file : files)
    {
        gcmd.m_respond_raw(file.name + " " + std::to_string(file.info.st_size / 1024) + "KB" + " " + std::to_string(file.info.st_mtim.tv_sec));
    }
    gcmd.m_respond_raw("End file list\r\n");
}

void VirtualSD::cmd_M21(GCodeCommand &gcmd)
{
    // Initialize SD card
    if (fopen(m_sdcard_dirname.c_str(), "r") == NULL)
    {
    }
    gcmd.m_respond_raw("SD card ok");
}

void VirtualSD::cmd_M23(GCodeCommand &gcmd)
{
    // Select SD file
    if (is_work_active())
    {
        LOG_E("SD busy\n");
        // std::cout << "SD busy" << std::endl;
        return;
        // raise m_gcode.error("SD busy")
    }
    _reset_file();
    std::string orig = gcmd.get_commandline();

    std::string filename = split(orig.substr(orig.find("M23") + 4), " ")[0];
    if (filename.find("*") < filename.length())
        filename = filename.substr(0, filename.find("*"));
    if (startswith(filename, "/"))
        filename = filename.substr(1);
    _load_file(gcmd, filename);
}

void VirtualSD::_load_file(GCodeCommand &gcmd, std::string filename, bool check_subdirs)
{
    // 确保之前的文件和缓存被清理
    if (m_current_file.is_open())
    {
        m_current_file.close();
    }
    m_lines.clear();
    m_partial_input.clear();
    m_next_file_position = 0;
    
    m_sdcard_print_file_src = gcmd.get_int("src", SDCARD_PRINT_FILE_SRC_LOCAL);
    m_taskid = gcmd.get_string("taskid", "0");

    if (!m_current_file.is_open())
    {
        m_current_file.open(filename.c_str(), ios::binary);
        if (!m_current_file.is_open())
        {
            LOG_E("Failed to open file %s\n", filename.c_str());
            return;
        }
        
        // 重置打印相关状态
        Printer::GetInstance()->m_printer_extruder->reset_extruder();
        Printer::GetInstance()->m_printer_extruder->m_start_print_flag = true;
        
        // 获取文件大小
        m_current_file.seekg(0, std::ios::end);
        int fsize = m_current_file.tellg();
        m_current_file.seekg(0, std::ios::beg);
        
        // 设置文件信息
        gcmd.m_respond_raw("File opened:" + filename + " Size:" + std::to_string(fsize));
        gcmd.m_respond_raw("File selected");
        m_current_file_name = filename;
        m_file_position = 0;
        m_file_size = fsize;
        
        // 更新打印状态
        Printer::GetInstance()->m_print_stats->set_current_file(filename, m_sdcard_print_file_src, m_taskid);
        Printer::GetInstance()->m_gcode_io->single_command("SAVE_VARIABLE VARIABLE=virtual_sd_stats-filename VALUE=" + (filename));
        Printer::GetInstance()->m_gcode_io->single_command("SAVE_VARIABLE VARIABLE=virtual_sd_stats-m_file_size VALUE=" + to_string(m_file_size));
    }
}

void VirtualSD::cmd_M24(GCodeCommand &gcmd)
{
    // Start/resume SD print
    do_resume();
}

void VirtualSD::cmd_M25(GCodeCommand &gcmd)
{
    // Pause SD print
    do_pause();
}

void VirtualSD::cmd_M26(GCodeCommand &gcmd)
{
    // Set SD position
    if (is_work_active())
    {
        LOG_E("SD busy\n");
        // std::cout << "SD busy" << std::endl;
        return;
        // raise m_gcode.error("SD busy")
    }
    int pos = gcmd.get_int("S", INT32_MIN, 0);
    m_file_position = pos;
}

void VirtualSD::cmd_M27(GCodeCommand &gcmd)
{
    // Report SD print status
    if (!m_current_file.is_open())
    {
        gcmd.m_respond_raw("Not SD printing.");
        return;
    }
    gcmd.m_respond_raw("SD printing byte " + std::to_string(m_file_position) + "/" + std::to_string(m_file_size));
}

void VirtualSD::cmd_M28(GCodeCommand &gcmd)
{
    std::string file_path = gcmd.get_string("f", "");
    M28_fd = open(file_path.c_str(), O_RDWR);
    if (M28_fd == -1)
    {
        M28_fd = open(file_path.c_str(), O_RDWR | O_CREAT, 0600); // O_CREAT 表示如果指定文件不存在，则创建这个文件。0600表示权限
        if (M28_fd > 0)
        {
            printf("create filed sucesse!\n");
        }
    }
}

void VirtualSD::cmd_M29(GCodeCommand &gcmd)
{
    std::string file_path = gcmd.get_string("f", "");
    close(M28_fd);
}

void VirtualSD::cmd_M30(GCodeCommand &gcmd)
{
    std::string file_path = gcmd.get_string("f", "");
    file_path = m_sdcard_dirname + file_path;
    if (!remove(file_path.c_str()))
        LOG_I("delete filed sucesse! %s\n", file_path.c_str());
}

int VirtualSD::get_file_position()
{
    return m_next_file_position;
}

void VirtualSD::set_file_position(int pos)
{
    m_next_file_position = pos;
}

bool VirtualSD::is_cmd_from_sd()
{
    return m_cmd_from_sd;
}

void VirtualSD::load_current_layer(std::string& line, std::string& next_line) {
    const bool is_cura_or_elegoo = (m_file_slicer == CURA_SLICER || m_file_slicer == ELEGOO_SLICER);
    const std::string layer_keyword = is_cura_or_elegoo ? "LAYER:" : "AFTER_LAYER_CHANGE";

    const size_t comment_pos = line.find(';');
    if (comment_pos == std::string::npos) return;

    // 提取注释内容（移除前导空格）
    std::string comment = line.substr(comment_pos + 1);
    line.resize(comment_pos);

    // 检查层标记
    const size_t keyword_pos = comment.find(layer_keyword);
    if (keyword_pos != 0) return;
    // LOG_I("current_line = %s，keyword_pos = %d\n", comment.c_str(), keyword_pos);

    int layer_number = -1;
    bool extraction_success = false;

    // 分层处理不同切片软件
    if (is_cura_or_elegoo) 
    {
        // CURA/ELEGOO：从注释中提取层号
        const size_t num_start = comment.find_first_of("0123456789", keyword_pos + layer_keyword.size());
        if (num_start != std::string::npos) 
        {
            const size_t num_end = comment.find_first_not_of("0123456789", num_start);
            const std::string number_str = comment.substr(
                num_start,
                (num_end == std::string::npos) ? std::string::npos : num_end - num_start
            );
            // LOG_I("current_number = %s\n", number_str.c_str());
            try 
            {
                layer_number = std::stoi(number_str);
                extraction_success = (layer_number >= 0);
            } catch (const std::exception& e) 
            {
                LOG_E("Layer number conversion failed: %s\n", e.what());
            }
        }
    } 
    else 
    {
        // 其他切片软件：从下一行提取层号
        const size_t num_start = next_line.find_first_not_of(' ');
        if (num_start != std::string::npos) 
        {
            const size_t num_end = next_line.find_first_of(' ', num_start);
            const std::string number_str = next_line.substr(
                num_start,
                (num_end == std::string::npos) ? std::string::npos : num_end - num_start
            );
            // LOG_I("next_line = %s，number = %s\n", next_line.c_str(), number_str.c_str());
            try 
            {
                layer_number = std::stoi(number_str) + 1; // 层号+1处理
                extraction_success = (layer_number >= 0);
            } catch (const std::exception& e) 
            {
                LOG_E("Next line layer conversion failed: %s\n", e.what());
            }
        }
    }

    // 更新打印机状态
    auto& printer = *Printer::GetInstance();
    if (!extraction_success) 
    {
        layer_number = printer.m_print_stats->get_status(get_monotonic(), nullptr).current_layer;
    }
    printer.m_gcode_io->single_command("SET_PRINT_STATS_INFO CURRENT_LAYER=%d", layer_number);
}

// Background work timer
double VirtualSD::work_handler(double eventtime)
{
    // 检查是否已经在handler中
    if (m_in_work_handler)
    {
        return Printer::GetInstance()->m_reactor->m_NEVER;
    }

    // 设置标记
    m_in_work_handler = true;

    // 检查是否需要暂停
    if (m_must_pause_work || !m_is_active)
    {
        if (is_work_active())
        {
            Printer::GetInstance()->m_reactor->delay_unregister_timer(m_work_timer);
        }
        m_in_work_handler = false; // 清除标记
        return Printer::GetInstance()->m_reactor->m_NEVER;
    }

    m_cancel = false;
    // logging.info("Starting SD card print (position %d)", m_file_position)
    if (is_work_active())
    {
        Printer::GetInstance()->m_reactor->delay_unregister_timer(m_work_timer);
    }
    // std::cout << "m_file_position = " << m_file_position << std::endl;

    Printer::GetInstance()->m_print_stats->note_start();
    bool already_started = false;
    // ReactorMutex* gcode_mutex = Printer::GetInstance()->m_gcode->get_mutex();
    // std::string partial_input = "";
    // std::vector<std::string> lines;
    // std::string error_message;
    // std::cout << "---------------sleep-------start---------------" << std::endl;
    // system("free");

    // 循环读取文件
    while (!m_must_pause_work)
    {
        // time_t start_time = time((time_t *)NULL);
        if (m_lines.size() == 0)
        {
// Read more data
#if 0
            char data[4096] = {0};
            if (m_current_file.is_open() && m_current_file.eof())
            {
                // End of file
                m_current_file.close();
                // logging.info("Finished SD card print")
                Printer::GetInstance()->m_gcode->respond_raw("Done printing file");
                break;
            }
            m_current_file.read(data, 4096);
            if (!m_current_file.fail())
            {
                m_lines = split(data, "\n");
                m_lines[0] = m_partial_input + m_lines[0];
                m_partial_input = m_lines.back();
                m_lines.pop_back();
            }
#else
            std::string command_line;
            int read_len = 0;
            // 打印完成
            if (m_current_file.is_open() && m_current_file.eof())
            {
                m_current_file.close();
                Printer::GetInstance()->m_gcode->respond_raw("Done printing file");
                break;
            }

            // 读取4K数据
            while (read_len < 4096 && !m_current_file.eof())
            {
                getline(m_current_file, command_line);
                read_len = m_current_file.tellg() - m_file_position;
                m_lines.push_back(command_line);
            }
#endif

            if (!m_lines.empty())
                std::reverse(m_lines.begin(), m_lines.end());
            Printer::GetInstance()->m_reactor->pause(get_monotonic());
            continue;
        }

        // 刷新定时器
        if (Printer::GetInstance()->m_manual_sq_require || Printer::GetInstance()->m_serial_sq_require || Printer::GetInstance()->m_highest_priority_sq_require)
        {
            Printer::GetInstance()->m_reactor->update_timer(Printer::GetInstance()->m_command_controller->ui_command_timer, Printer::GetInstance()->m_reactor->m_NOW);
            // Printer::GetInstance()->m_reactor->update_timer(Printer::GetInstance()->m_command_controller->serial_command_timer, Printer::GetInstance()->m_reactor->m_NOW);
            Printer::GetInstance()->m_reactor->update_timer(Printer::GetInstance()->m_command_controller->highest_priority_cmd_timer, Printer::GetInstance()->m_reactor->m_NOW);
            Printer::GetInstance()->m_reactor->pause(get_monotonic());
            continue;
        }

        if (Printer::GetInstance()->m_fan_cmd_sq_require)
        {
            Printer::GetInstance()->m_reactor->update_timer(Printer::GetInstance()->m_command_controller->fan_cmd_timer, Printer::GetInstance()->m_reactor->m_NOW);
            Printer::GetInstance()->m_reactor->pause(get_monotonic());
            continue;
        }

        for (auto heater : Printer::GetInstance()->m_verify_heaters)
        {
            if (heater.second->m_check_timer != nullptr && heater.second->m_check_timer->m_waketime < get_monotonic())
            {
                heater.second->m_check_timer->m_waketime = heater.second->m_check_timer->m_callback(get_monotonic());
            }
        }
        // 切料暂停导致丢失的命令
        if (m_pending_command.size())
        {
            Printer::GetInstance()->m_gcode_io->single_command(m_pending_command.front().get_commandline());
            m_pending_command.pop_front();
            continue;
        }

        // Printer::GetInstance()->m_print_stats->m_timer->m_callback(get_monotonic());
        // Dispatch command
        m_cmd_from_sd = true;
        std::string next_line = "";
        if (m_lines.size() >= 1)
        {
            next_line = (m_lines.size() > 1) ? m_lines[m_lines.size() - 2] : "";
        }
        std::string line = m_lines.back();
        m_lines.pop_back(); // 并不会释放内存   但会被重新使用

        // std::cout << "line = " << line << std::endl;
        // std::cout << "m_file_position = " << m_file_position << std::endl;
        int next_file_position = m_file_position + line.size() + 1;
        m_next_file_position = next_file_position;
        // std::cout << ">>>>>>>>file_m_file_position = " << m_file_position << std::endl;

        load_current_layer(line, next_line);

        // 判断实际开始
        if (already_started == false)
        {
            Printer::GetInstance()->m_print_stats->update_filament_usage(get_monotonic());
            if (fabs(Printer::GetInstance()->m_print_stats->get_status(get_monotonic(), NULL).filament_used) > 1e-15)
            {
                Printer::GetInstance()->m_print_stats->note_actual_start();
                already_started = true;
            }
        }

        // 执行命令
        if (line != "" || Printer::GetInstance()->m_gcode->is_traditional_gcode(line) || strstr(line.c_str(), "ROOT"))
        {
            if (!custom_zero && line.find("G28") != std::string::npos)
            {
                custom_zero = true;
                line = "CUSTOM_ZERO";
            }
            std::vector<std::string> line_str = {line};
            // static uint64_t last_time_count = 0;
            // static int last_file_position = 0;
            // if (hl_get_utc_ms() - last_time_count >= 1000)
            // {
            //     LOG_I("file_position / sec: %d\n", m_file_position - last_file_position);
            //     last_time_count = hl_get_utc_ms();
            //     last_file_position = m_file_position;
            // }
            Printer::GetInstance()->m_gcode->run_script(line_str);
        }

        m_cmd_from_sd = false;
        m_file_position = m_next_file_position;
        // Do we need to skip around
        if (m_next_file_position != next_file_position)
        {
            m_current_file.seekg(m_file_position, std::ios::beg);
            m_lines = std::vector<std::string>();
            m_partial_input = "";
        }
        // time_t end_time = time((time_t *)NULL);
        // Printer::GetInstance()->m_print_stats->m_already_print_time_seconds += ((int)end_time - (int)start_time);
    }
    // system("free");
    // std::cout << "---------------sleep-------end---------------" << std::endl;
    // logging.info("Exiting SD card print (position %d)", m_file_position)
    // m_cmd_from_sd = false;
    Printer::GetInstance()->m_printer_para->is_root = false;
    // 错误
    // if (error_message != "") //todo
    // {
    // Printer::GetInstance()->m_print_stats->note_error(error_message);
    // }
    // 暂停
    if (m_current_file.is_open())
    {
        // 点击暂停时执行两遍note_pause,屏蔽此处
        // Printer::GetInstance()->m_print_stats->note_pause();
    }
    // 完成
    else
    {
        custom_zero = false;
        m_is_active = false;
        if (!m_cancel)
            Printer::GetInstance()->m_print_stats->note_complete();
        else
            m_cancel = false;
    }
    m_in_work_handler = false;
    return Printer::GetInstance()->m_reactor->m_NEVER;
}

bool VirtualSD::is_work_active()
{
    return Printer::GetInstance()->m_reactor->is_timer_valid(m_work_timer);
}
