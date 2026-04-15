#include "bed_mesh.h"
#include "Define.h"
#include "klippy.h"
#include "my_string.h"
#include "Define_config_path.h"
#include <cctype>
#define LOG_TAG "bed_mesh"
#undef LOG_LEVEL
#define LOG_LEVEL LOG_DEBUG
#include "log.h"
#define PROFILE_VERSION 1
#define PROFILE_PREFIX_NAME "besh_profile"
#define PROFILE_PREFIX_SUFFIX_NAME "default"
#define PROFILE_NAMED_PREFIX "named_"
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

static std::string build_named_profile_section(const std::string &profile_name)
{
    return std::string(PROFILE_PREFIX_NAME) + "_" + PROFILE_NAMED_PREFIX + profile_name;
}
/**
 * @brief 这个函数有四个参数，其中a和b是要比较的两个double类型的数值。
 *
 * @param a
 * @param b
 * @param rel_tol 相对误差的容忍度 默认值是1e-09
 * @param abs_tol 绝对误差的容忍度 默认值是0
 * @return true 足够接近
 * @return false 说明a和b之间的差距大于相对误差和绝对误差中的较大值，即不足够接近
 */
bool isclose(double a, double b, double rel_tol = 1e-09, double abs_tol = 0.0f)
{
    return (fabs(a - b) <= std::max(rel_tol * std::max(fabs(a), fabs(b)), abs_tol));
}

// return true if a coordinate is within the region specified by min_c and max_c
bool within(std::vector<double> coord, std::vector<double> min_c, std::vector<double> max_c, double tol = 0.0)
{
    return (max_c[0] + tol) >= coord[0] && coord[0] >= (min_c[0] - tol) && (max_c[1] + tol) >= coord[1] && coord[1] >= (min_c[1] - tol);
}

// Constrain value between min and max
double constrain(double val, double min_val, double max_val)
{
    return std::min(max_val, std::max(min_val, val));
}

// Linear interpolation between two values
double lerp(double t, double v0, double v1)
{
    return (1. - t) * v0 + t * v1;
}

std::vector<double> parse_pair(std::string section_name, std::string option, std::string default_params, bool check = true, double minval = DBL_MIN, double maxval = DBL_MAX)
{
    std::string config_params = Printer::GetInstance()->m_pconfig->GetString(section_name, option, default_params);
    std::vector<double> values;
    strip(config_params);
    std::istringstream iss(config_params); // 输入流
    std::string token_str;
    while (getline(iss, token_str, ','))
    {
        values.push_back(atof(token_str.c_str()));
    }
    if (check && values.size() != 2)
    {
        std::cout << "bed_mesh: malformed value" << std::endl;
    }
    else if (values.size() == 1)
    {
        values.push_back(values[0]);
    }
    if (minval != DBL_MIN)
    {
        if (values[0] < minval || values[1] < minval)
        {
            std::cout << "bed_mesh must have a minimum" << std::endl;
        }
    }
    if (maxval != DBL_MAX)
    {
        if (values[0] > maxval || values[1] > maxval)
        {
            std::cout << "bed_mesh must have a maximum" << std::endl;
        }
    }
    return values;
}

// retreive commma separated pair from config
BedMesh::BedMesh(std::string section_name)
{
    m_FADE_DISABLE = 0x7FFFFFFF;
    Printer::GetInstance()->register_event_handler("klippy:connect:BedMesh", std::bind(&BedMesh::handle_connect, this));
    m_last_position = {0., 0., 0., 0.};
    m_horizontal_move_z = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "horizontal_move_z", DEFAULT_BED_MESH_HORIZONTAL_MOVE_Z); // 头部应该被命令移动到的高度（以毫米为单位）就在开始探测操作之前。默认值为 5。
    m_fade_start = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "fade_start", 1.0f);
    m_fade_end = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "fade_end", 0.0f); // 默认禁用网格淡出
    m_multi_bed_mesh_enable = Printer::GetInstance()->m_pconfig->GetBool(section_name, "multi_bed_mesh_enable", false);
    m_fade_dist = m_fade_end - m_fade_start;
    m_current_mesh_index = 0;
    m_platform_material = Printer::GetInstance()->m_pconfig->GetString(section_name, "platform_material", "standard");

    m_bmc = new BedMeshCalibrate(section_name, this);
    // ????
    if (m_fade_dist <= 0.0f)
    {
        m_fade_start = m_FADE_DISABLE;
        m_fade_end = m_FADE_DISABLE;
    }
    m_log_fade_complete = false;
    m_base_fade_target = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "fade_target", DBL_MIN);
    m_fade_target = 0.0f;
    m_splitter = new MoveSplitter(section_name);
    // setup persistent storage
    Printer::GetInstance()->load_object("gcode_move");
    m_pmgr = new ProfileManager("besh_profile_default", this);
    // register gcodes
    cmd_BED_MESH_OUTPUT_help = "Retrieve interpolated grid of probed z-points";
    cmd_BED_MESH_MAP_help = "Serialize mesh and output to terminal";
    cmd_BED_MESH_CLEAR_help = "Clear the Mesh so no z-adjustment is made";
    cmd_BED_MESH_OFFSET_help = "Add X/Y offsets to the mesh lookup";
    Printer::GetInstance()->m_gcode->register_command("BED_MESH_OUTPUT", std::bind(&BedMesh::cmd_BED_MESH_OUTPUT, this, std::placeholders::_1), false, cmd_BED_MESH_OUTPUT_help);
    Printer::GetInstance()->m_gcode->register_command("BED_MESH_MAP", std::bind(&BedMesh::cmd_BED_MESH_MAP, this, std::placeholders::_1), false, cmd_BED_MESH_MAP_help);
    Printer::GetInstance()->m_gcode->register_command("BED_MESH_CLEAR", std::bind(&BedMesh::cmd_BED_MESH_CLEAR, this, std::placeholders::_1), false, cmd_BED_MESH_CLEAR_help);
    Printer::GetInstance()->m_gcode->register_command("BED_MESH_OFFSET", std::bind(&BedMesh::cmd_BED_MESH_OFFSET, this, std::placeholders::_1), false, cmd_BED_MESH_OFFSET_help);
    Printer::GetInstance()->m_gcode->register_command("BED_MESH_APPLICATIONS", std::bind(&BedMesh::cmd_BED_MESH_APPLICATIONS, this, std::placeholders::_1), false);
    Printer::GetInstance()->m_gcode->register_command("BED_MESH_SET_INDEX", std::bind(&BedMesh::cmd_BED_MESH_SET_INDEX, this, std::placeholders::_1), false);
    Printer::GetInstance()->m_gcode->register_command("SET_ACTIVE_MESH", std::bind(&BedMesh::cmd_SET_ACTIVE_MESH, this, std::placeholders::_1), false);
    Printer::GetInstance()->m_gcode->register_command("CALIBRATE_BED_SURFACE", std::bind(&BedMesh::cmd_CALIBRATE_BED_SURFACE, this, std::placeholders::_1), false);

    // Register transform
    Printer::GetInstance()->m_gcode_move->set_move_transform(std::bind(&BedMesh::move, this, std::placeholders::_1, std::placeholders::_2));
    Printer::GetInstance()->m_gcode_move->set_get_position_transform(std::bind(&BedMesh::get_position, this));
}

BedMesh::~BedMesh()
{
}

void BedMesh::handle_connect()
{
    m_bmc->print_generated_points();
    m_pmgr->initialize();
    std::string active_profile = get_active_mesh_profile();
    if (active_profile != "")
    {
        if (!m_pmgr->load_profile_by_name(active_profile))
        {
            LOG_E("Failed to load active bed mesh profile '%s' during startup\n", active_profile.c_str());
        }
    }
}

void BedMesh::set_mesh(ZMesh *mesh)
{
    if (mesh != nullptr && m_fade_end != m_FADE_DISABLE)
    {
        m_log_fade_complete = true;
        if (m_base_fade_target == 0.0)
        {
            m_fade_target = mesh->m_avg_z;
        }
        else
        {
            m_fade_target = m_base_fade_target;
            double min_z = mesh->get_z_range()[0];
            double max_z = mesh->get_z_range()[1];
            if (!(min_z <= m_fade_target && m_fade_target <= max_z) && m_fade_target != 0.0)
            {
                // fade target is non-zero, out of mesh range
                double err_target = m_fade_target;
                m_z_mesh = nullptr;
                m_fade_target = 0.;
                printf("bed_mesh: ERROR, fade_target lies outside of mesh z range min: %.4f, max: %.4f, fade_target: %.4f\n", min_z, max_z, err_target);
            }
        }
        double min_z = mesh->get_z_range()[0];
        double max_z = mesh->get_z_range()[1];

        if (m_fade_dist <= std::max(fabs(min_z), fabs(max_z)))
        {
            m_z_mesh = nullptr;
            m_fade_target = 0.;
            printf("bed_mesh:  Mesh extends outside of the fade range, \
                please see the fade_start and fade_end options in \
                example-extras.cfg. fade distance: %.2f mesh min: %.4f \
                mesh max: %.4f\n",
                   m_fade_dist, min_z, max_z);
        }
    }
    else
    {
        m_fade_target = 0.0f;
    }
    m_z_mesh = mesh;
    m_splitter->initialize(mesh, m_fade_target);
    // cache the current position before a transform takes place
    Printer::GetInstance()->m_gcode_move->reset_last_position();
}

double BedMesh::get_z_factor(double z_pos)
{
    // LOG_D("BedMesh::get_z_factor:%f m_fade_end:%f m_fade_start:%f m_fade_dist:%f\n",z_pos,m_fade_end,m_fade_start,m_fade_dist);
    if (z_pos >= m_fade_end)
        return 0.;
    else if (z_pos >= m_fade_start)
        return (m_fade_end - z_pos) / m_fade_dist;
    else
        return 1.;
    // get_z_factor:0.200000 m_fade_end:2147483647.000000 m_fade_start:2147483647.000000 m_fade_dist:-1.000000
}

std::vector<double> BedMesh::get_position()
{
    // Return last, non-transformed position
    if (m_z_mesh == nullptr) // 没有床网参数
    {
        // No mesh calibrated, so send toolhead position
        m_last_position = Printer::GetInstance()->m_tool_head->get_position();
        m_last_position[2] -= m_fade_target;
    }
    else
    {
        // return current position minus the current z-adjustment
        std::vector<double> pos = Printer::GetInstance()->m_tool_head->get_position();
        double max_adj = m_z_mesh->calc_z(pos[0], pos[1]);
        double factor = 1.0f;
        double z_adj = max_adj - m_fade_target;
        if (std::min(pos[2], (pos[2] - max_adj)) >= m_fade_end)
        {
            // Fade out is complete, no factor
            factor = 0.0f;
        }
        else if (std::max(pos[2], (pos[2] - max_adj)) >= m_fade_start)
        {
            // Likely in the process of fading out adjustment.
            // Because we don't yet know the gcode z position, use
            // algebra to calculate the factor from the toolhead pos
            factor = ((m_fade_end + m_fade_target - pos[2]) / (m_fade_dist - z_adj));
            factor = constrain(factor, 0., 1.);
        }
        double final_z_adj = factor * z_adj + m_fade_target;
        m_last_position[0] = pos[0];
        m_last_position[1] = pos[1];
        m_last_position[2] = pos[2] - final_z_adj;
        // std::cout << "factor:"<< factor << " m_fade_target:" << m_fade_target << "final_z_adj:" << final_z_adj << std::endl;
        m_last_position[3] = pos[3];
    }
    return m_last_position;
}
// 如果z_mesh为None或者factor为False，则表示没有进行网格校准或者当前位置不在网格内，直接移动打印头到新位置上。
// 如果有网格校准，则通过splitter.build_move()方法将当前位置和新位置之间的移动分割成多个小的移动。在每个小的移动中，
// 通过计算当前位置的Z因子来调整打印头的高度，从而保持平稳的打印。这些小的移动通过self.toolhead.move()方法一个一个地执行。
// 最后，将当前位置更新为新位置。
bool BedMesh::move(std::vector<double> newpos, double speed)
{
    double factor = get_z_factor(newpos[2]); // 获取当前位置的Z因子，目前的不太理解。
    // LOG_D("BedMesh::move factor:%f\n",factor);
    bool ret;
    // Printer::GetInstance()->m_tool_head->m_stop_bed_mesh = false;
    if (m_z_mesh == nullptr || !factor)
    {
        // LOG_D("BedMesh::move m_z_mesh == nullptr || !factor\n");
        // No mesh calibrated, or mesh leveling phased out.
        if (m_log_fade_complete)
        {
            m_log_fade_complete = false;
            printf("bed_mesh fade complete: Current Z: %.4f fade_target: %.4f \n", newpos[2], m_fade_target);
        }
        double pos[4] = {newpos[0], newpos[1], newpos[2] + m_fade_target, newpos[3]};
        ret = Printer::GetInstance()->m_tool_head->move1(pos, speed);
    }
    else
    {
        // LOG_D("BedMesh::move build_move factor:%f\n",factor);
        // 检查是否超出软限位 若超出则移动到限位位置后不再移动
        check_move(m_last_position, newpos);
        Printer::GetInstance()->m_gcode_move->m_last_position = newpos;
        m_splitter->build_move(m_last_position, newpos, factor);
        while (false == m_splitter->m_traverse_complete)
        {
            // if (Printer::GetInstance()->m_tool_head->m_stop_bed_mesh)
            // {
            //     Printer::GetInstance()->m_tool_head->m_stop_bed_mesh = false;
            //     break;
            // }
            std::vector<double> split_move = m_splitter->split();
            if (split_move[0] != DBL_MIN)
            {
                double pos[4] = {split_move[0], split_move[1], split_move[2], split_move[3]};
                // std::cout << "pos : " << pos[0] << " " << pos[1] << " " << pos[2] << " " << pos[3] << std::endl;
                ret = Printer::GetInstance()->m_tool_head->move1(pos, speed);
            }
            else
                printf("Mesh Leveling: Error splitting move \n");
        }
    }
    // Printer::GetInstance()->m_tool_head->m_stop_bed_mesh = false;
    m_last_position = newpos;
    return ret;
}

bool BedMesh::check_move(std::vector<double> last_pos, std::vector<double> &newpos)
{
    if (fabs(sqrt(pow((newpos[0] - last_pos[0]), 2) + pow((newpos[1] - last_pos[1]), 2) + pow((newpos[2] - last_pos[2]), 2))) <= 1e-15)
    {
        return true;
    }
    if (Printer::GetInstance()->m_tool_head->m_kin->check_endstops(newpos))
    {
        return true;
    }
    return true;
}

/**
 * @description: 
 * @author:  
 * @param {GCodeCommand} &gcmd
 * @return {*}
 */
void BedMesh::cmd_BED_MESH_SET_INDEX(GCodeCommand &gcmd)
{
    int index = gcmd.get_int("INDEX", 0);   
    m_platform_material = gcmd.get_string("TYPE", m_platform_material);
    if (m_multi_bed_mesh_enable && index)
    {
        m_current_mesh_index = index;
    }
    else
    {
        m_current_mesh_index = 0;
    }
    if (m_platform_material == "standard")
    {
        Printer::GetInstance()->m_gcode_io->single_command("SET_GCODE_OFFSET Z=0 MOVE=0 MOVE_SPEED=5.0");
        Printer::GetInstance()->m_gcode_io->single_command("M8233 S%.3f P1", Printer::GetInstance()->m_pconfig->GetDouble("bed_mesh", "standard_z_offset", Printer::GetInstance()->m_bed_mesh_probe->m_z_offset));
        // Printer::GetInstance()->m_probe->m_z_offset = 0;
        // Printer::GetInstance()->m_bed_mesh_probe->m_z_offset = 0;
        Printer::GetInstance()->m_pconfig->SetValue("bed_mesh", "platform_material", "standard");
        Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
        Printer::GetInstance()->m_strain_gauge->m_cfg->m_fix_z_offset = -Printer::GetInstance()->m_unmodifiable_cfg->GetDouble("strain_gauge", "standard_fix_z_offset", Printer::GetInstance()->m_strain_gauge->m_cfg->m_fix_z_offset);
    }
    else if (m_platform_material == "enhancement")
    {
        Printer::GetInstance()->m_gcode_io->single_command("SET_GCODE_OFFSET Z=0 MOVE=0 MOVE_SPEED=5.0");
        Printer::GetInstance()->m_gcode_io->single_command("M8233 S%.3f P1", Printer::GetInstance()->m_pconfig->GetDouble("bed_mesh", "enhancement_z_offset", Printer::GetInstance()->m_bed_mesh_probe->m_z_offset));
        // Printer::GetInstance()->m_probe->m_z_offset = 0;
        // Printer::GetInstance()->m_bed_mesh_probe->m_z_offset = 0;
        Printer::GetInstance()->m_pconfig->SetValue("bed_mesh", "platform_material", "enhancement");
        Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
        Printer::GetInstance()->m_strain_gauge->m_cfg->m_fix_z_offset = -Printer::GetInstance()->m_unmodifiable_cfg->GetDouble("strain_gauge", "enhancement_fix_z_offset", Printer::GetInstance()->m_strain_gauge->m_cfg->m_fix_z_offset);
    }
    m_bmc->set_mesh_config();
    // m_bmc->print_generated_points();
    m_pmgr->load_profile(PROFILE_PREFIX_SUFFIX_NAME);
}

std::string BedMesh::get_active_mesh_profile()
{
    return Printer::GetInstance()->m_pconfig->GetString("bed_mesh", ACTIVE_MESH_PROFILE_OPTION, "");
}

void BedMesh::set_active_mesh_profile(const std::string &profile_name)
{
    Printer::GetInstance()->m_pconfig->SetValue("bed_mesh", ACTIVE_MESH_PROFILE_OPTION, profile_name);
    Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
    Printer::GetInstance()->m_pconfig->WriteI_specified_Ini(USER_CONFIG_PATH, "bed_mesh", {ACTIVE_MESH_PROFILE_OPTION});
}

void BedMesh::cmd_SET_ACTIVE_MESH(GCodeCommand &gcmd)
{
    std::string mesh_name = normalize_mesh_profile_name(gcmd.get_string("MESH", ""));
    if (mesh_name == "")
    {
        gcmd.m_respond_info("SET_ACTIVE_MESH requires MESH=<profile_name>", true);
        return;
    }
    if (!m_pmgr->load_profile_by_name(mesh_name))
    {
        gcmd.m_respond_info("Unable to load mesh profile [" + mesh_name + "]", true);
        return;
    }
    set_active_mesh_profile(mesh_name);
    gcmd.m_respond_info("Active mesh set to [" + mesh_name + "]", true);
}

void BedMesh::cmd_CALIBRATE_BED_SURFACE(GCodeCommand &gcmd)
{
    std::string bed_name = normalize_mesh_profile_name(gcmd.get_string("BED", ""));
    std::string surface_name = normalize_mesh_profile_name(gcmd.get_string("SURFACE", ""));
    int bed_temp = gcmd.get_int("TEMP", -1);
    int soak_seconds = gcmd.get_int("SOAK", 0, 0);
    bool should_home = gcmd.get_int("HOME", 1, 0, 1) == 1;
    std::string profile_name = normalize_mesh_profile_name(gcmd.get_string("PROFILE", ""));

    if (bed_temp < 0)
    {
        gcmd.m_respond_info("CALIBRATE_BED_SURFACE requires TEMP=<bed_temp>", true);
        return;
    }
    if (profile_name == "")
    {
        if (bed_name == "" || surface_name == "")
        {
            gcmd.m_respond_info("CALIBRATE_BED_SURFACE requires BED= and SURFACE= when PROFILE is not provided", true);
            return;
        }
        profile_name = bed_name + "_" + surface_name + "_" + to_string(bed_temp);
    }

    if (should_home)
    {
        Printer::GetInstance()->m_gcode_io->single_command("G28");
    }
    Printer::GetInstance()->m_gcode_io->single_command("M140 S" + to_string(bed_temp));
    Printer::GetInstance()->m_gcode_io->single_command("M190 S" + to_string(bed_temp));
    if (soak_seconds > 0)
    {
        Printer::GetInstance()->m_gcode_io->single_command("G4 S" + to_string(soak_seconds));
    }
    Printer::GetInstance()->m_gcode_io->single_command("BED_MESH_CLEAR");
    Printer::GetInstance()->m_gcode_io->single_command("BED_MESH_CALIBRATE");
    Printer::GetInstance()->m_gcode_io->single_command("BED_MESH_PROFILE SAVE=" + profile_name);
    Printer::GetInstance()->m_gcode_io->single_command("SET_ACTIVE_MESH MESH=" + profile_name);
}

void BedMesh::get_status(double eventtime) // 暂时没发现这部分代码作用
{
    // status = {
    //     "profile_name": "",
    //     "mesh_min": (0., 0.),
    //     "mesh_max": (0., 0.),
    //     "probed_matrix": [[]],
    //     "mesh_matrix": [[]]
    // }
    // if self.z_mesh is not None:
    //     params = self.z_mesh.get_mesh_params()
    //     mesh_min = (params['min_x'], params['min_y'])
    //     mesh_max = (params['max_x'], params['max_y'])
    //     probed_matrix = self.z_mesh.get_probed_matrix()
    //     mesh_matrix = self.z_mesh.get_mesh_matrix()
    //     status['profile_name'] = self.pmgr.get_current_profile()
    //     status['mesh_min'] = mesh_min
    //     status['mesh_max'] = mesh_max
    //     status['probed_matrix'] = probed_matrix
    //     status['mesh_matrix'] = mesh_matrix
    // return status  //---??---
}
void BedMesh::update_status() // 暂时没发现这部分代码作用
{
    // if (m_z_mesh != nullptr)
    // {
    //     struct mesh_config params = m_z_mesh->get_mesh_params();
    //     m_mesh_min = (params['min_x'], params['min_y']);
    //     m_mesh_max = (params['max_x'], params['max_y']);
    //     m_probed_matrix = m_z_mesh.get_probed_matrix();
    //     m_mesh_matrix = m_z_mesh.get_mesh_matrix();
    //     status['profile_name'] = self.pmgr.get_current_profile();
    //     status['mesh_min'] = mesh_min;
    //     status['mesh_max'] = mesh_max;
    //     status['probed_matrix'] = probed_matrix;
    //     status['mesh_matrix'] = mesh_matrix;
    // }
}
ZMesh *BedMesh::get_mesh()
{
    return m_z_mesh;
}

void BedMesh::cmd_BED_MESH_OUTPUT(GCodeCommand &gcmd)
{
    if (gcmd.get_int("PGP", 0))
    {
        // Print Generated Points instead of mesh
        m_bmc->print_generated_points();
    }
    else if (m_z_mesh == nullptr)
    {
        printf("Bed has not been probed \n");
    }
    else
    {
        m_z_mesh->print_probed_matrix(NULL);             // 输出流端口注册机制没有移植
        m_z_mesh->print_mesh(NULL, m_horizontal_move_z); // 输出流端口注册机制没有移植
    }
}

void BedMesh::cmd_BED_MESH_MAP(GCodeCommand &gcmd)
{
    if (m_z_mesh != nullptr)
    {
        // params = m_z_mesh.get_mesh_params();
        // outdict = {
        //     'mesh_min': (params['min_x'], params['min_y']),
        //     'mesh_max': (params['max_x'], params['max_y']),
        //     'z_positions': m_z_mesh.get_probed_matrix()}
        // gcmd.respond_raw("mesh_map_output " + json.dumps(outdict))   //---??---bed_mesh
    }
    else
    {
        // gcmd.m_respond_info("Bed has not been probed"); //---??---bed_mesh
    }
}

void BedMesh::cmd_BED_MESH_CLEAR(GCodeCommand &gcmd)
{
    set_mesh(nullptr);
}

void BedMesh::cmd_BED_MESH_OFFSET(GCodeCommand &gcmd)
{
    if (m_z_mesh != nullptr)
    {
        std::vector<double> offsets;
        offsets[0] = gcmd.get_double("X", DBL_MIN);
        offsets[1] = gcmd.get_double("Y", DBL_MIN);
        m_z_mesh->set_mesh_offsets(offsets);
        Printer::GetInstance()->m_gcode_move->reset_last_position();
    }
    else
    {
        printf("No mesh loaded to offset\n");
    }
}

void BedMesh::cmd_BED_MESH_APPLICATIONS(GCodeCommand &gcmd)
{
    if (m_old_z_mesh == nullptr && m_z_mesh == nullptr)
    {
        return;
    }
    int enable = gcmd.get_int("ENABLE", 1);
    if (enable)
    {
        set_mesh(m_old_z_mesh);
    }
    else
    {
        m_old_z_mesh = get_mesh();
        set_mesh(nullptr);
    }
}

BedMeshCalibrate::BedMeshCalibrate(std::string section_name, BedMesh *bedmesh)
{
    m_ALGOS.push_back("lagrange");
    m_ALGOS.push_back("bicubic");
    m_orig_config.radius = 0.;
    m_radius = 0.;
    m_mesh_min.emplace_back(0);
    m_mesh_min.emplace_back(0);
    m_mesh_max.emplace_back(0);
    m_mesh_max.emplace_back(0);
    m_relative_reference_index = Printer::GetInstance()->m_pconfig->GetInt(section_name, "relative_reference_index", 0);
    m_faulty_regions;
    m_orig_config.rri = m_relative_reference_index;
    m_bedmesh = bedmesh;
    m_section_name = section_name;
    _init_mesh_config(m_section_name);
    _generate_points();
    m_orig_points = m_points;
    m_probe_helper = new ProbePointsHelper(m_section_name, std::bind(&BedMeshCalibrate::probe_finalize, this, std::placeholders::_1, std::placeholders::_2), _get_adjusted_points());
    m_probe_helper->minimum_points(3);
    m_probe_helper->use_xy_offsets(true);
    cmd_BED_MESH_CALIBRATE_help = "Perform Mesh Bed Leveling";
    Printer::GetInstance()->m_gcode->register_command("BED_MESH_CALIBRATE", std::bind(&BedMeshCalibrate::cmd_BED_MESH_CALIBRATE, this, std::placeholders::_1), false, cmd_BED_MESH_CALIBRATE_help);
}

BedMeshCalibrate::~BedMeshCalibrate()
{
}

void BedMeshCalibrate::set_mesh_config()
{
    _init_mesh_config(m_section_name);
    _generate_points();
    m_orig_points = m_points;
    std::vector<std::vector<double>> pts = _get_adjusted_points();
    m_probe_helper->update_probe_points(pts, 3);
}

void BedMeshCalibrate::_generate_points()
{
    int x_cnt = m_mesh_config.x_count;
    int y_cnt = m_mesh_config.y_count;
    double min_x = m_mesh_min[0];
    double min_y = m_mesh_min[1];
    double max_x = m_mesh_max[0];
    double max_y = m_mesh_max[1];
    double x_dist = (max_x - min_x) / (x_cnt - 1);
    double y_dist = (max_y - min_y) / (y_cnt - 1);
    // floor distances down to next hundredth
    x_dist = floor(x_dist * 100) / 100;
    y_dist = floor(y_dist * 100) / 100;
    if (x_dist <= 1. || y_dist <= 1.)
        std::cout << "bed_mesh: min/max points too close together" << std::endl;
    if (m_radius != 0)
    {
        // round bed, min/max needs to be recalculated
        y_dist = x_dist;
        int new_r = (x_cnt / 2) * x_dist;
        min_x = min_y = -new_r;
        max_x = max_y = new_r;
    }
    else
    {
        // rectangular bed, only re-calc max_x
        max_x = min_x + x_dist * (x_cnt - 1);
    }
    double pos_y = min_y;
    double pos_x = 0;
    std::vector<std::vector<double>> points;
    for (int i = 0; i < y_cnt; i++)
    {
        for (int j = 0; j < x_cnt; j++)
        {
            if (!(i % 2))
            {
                // move in positive directon
                pos_x = min_x + j * x_dist;
            }
            else
            {
                // move in negative direction
                pos_x = max_x - j * x_dist;
            }
            if (m_radius == 0)
            {
                // rectangular bed, append
                std::vector<double> point = {pos_x, pos_y};
                points.push_back(point);
            }
            else
            {
                // round bed, check distance from origin
                double dist_from_origin = sqrt(pos_x * pos_x + pos_y * pos_y);
                if (dist_from_origin <= m_radius)
                {
                    std::vector<double> point = {m_origin[0] + pos_x, m_origin[1] + pos_y};
                    points.push_back(point);
                }
            }
        }
        pos_y += y_dist;
    }
    m_points.assign(points.begin(), points.end());
    if (m_faulty_regions.size() == 0)
        return;
    // Check to see if any points fall within faulty regions
    double last_y = m_points[0][1];
    bool is_reversed = false;
    for (int i = 0; i < points.size(); i++)
    {
        std::vector<double> coord = points[i];
        if (!isclose(coord[1], last_y))
        {
            is_reversed = !is_reversed;
        }
        last_y = coord[1];
        std::vector<std::vector<double>> adj_coords;
        for (int j = 0; j < m_faulty_regions.size(); j++)
        {
            std::vector<double> min_c = m_faulty_regions[j][0];
            std::vector<double> max_c = m_faulty_regions[j][1];
            if (within(coord, min_c, max_c, 0.00001))
            {
                // Point lies within a faulty region
                std::vector<double> adj_coord1 = {min_c[0], coord[1]};
                std::vector<double> adj_coord2 = {coord[0], min_c[1]};
                std::vector<double> adj_coord3 = {coord[0], max_c[1]};
                std::vector<double> adj_coord4 = {max_c[0], coord[1]};
                adj_coords.push_back(adj_coord1);
                adj_coords.push_back(adj_coord2);
                adj_coords.push_back(adj_coord3);
                adj_coords.push_back(adj_coord4);
                if (is_reversed)
                {
                    // Swap first and last points for zig-zag pattern
                    adj_coords.front().swap(adj_coords.back());
                }
                break;
            }
        }

        if (adj_coords.size() == 0)
        {
            // coord is not located within a faulty region
            continue;
        }
        std::vector<std::vector<double>> valid_coords;
        // std::vector<double> ac = adj_coords[0];
        for (auto ac : adj_coords)
        {
            // make sure that coordinates are within the mesh boundary
            if (m_radius == 0)
            {
                std::vector<double> pos1 = {min_x, min_y};
                std::vector<double> pos2 = {max_x, max_y};
                if (within(ac, pos1, pos2, 0.000001))
                    valid_coords.push_back(ac);
            }
            else
            {
                double dist_from_origin = sqrt(ac[0] * ac[0] + ac[1] * ac[1]);
                if (dist_from_origin <= m_radius)
                    valid_coords.push_back(ac);
            }
        }
        if (valid_coords.size() == 0)
        {
            std::cout << "bed_mesh: Unable to generate coordinates for faulty region at index: " << i << std::endl;
        }
        m_substituted_indices[i] = valid_coords;
    }
}

void BedMeshCalibrate::print_generated_points()
{
    double x_offset = 0.;
    double y_offset = 0.;
    if (Printer::GetInstance()->m_bed_mesh_probe != nullptr)
    {
        x_offset = Printer::GetInstance()->m_bed_mesh_probe->get_offsets()[0];
        y_offset = Printer::GetInstance()->m_bed_mesh_probe->get_offsets()[1];
    }
    printf("bed_mesh: generated points\nIndex"
           " |  Tool Adjusted  |   Probe\n");
    for (int i = 0; i < m_points.size(); i++)
    {
        std::vector<double> point = m_points[i];
        std::vector<double> adj_pt = {point[0] - x_offset, point[1] - y_offset};
        std::vector<double> mesh_pt = {point[0], point[1]};
        printf("  %-4d| (%.1f, %.1f)  | (%.1f, %.1f)\n", i, adj_pt[0], adj_pt[1], mesh_pt[0], mesh_pt[1]);
    }

    if (m_relative_reference_index)
    {
        int rri = m_relative_reference_index;
        printf("bed_mesh: relative_reference_index %d is (%.2f, %.2f)\n", rri, m_points[rri][0], m_points[rri][1]);
    }

    if (m_substituted_indices.size() != 0)
    {
        printf("bed_mesh: faulty region points\n");
    }
}

void BedMeshCalibrate::_init_mesh_config(std::string section_name)
{
    // struct mesh_config m_mesh_config = m_mesh_config;
    // struct orig_config m_orig_config = m_orig_config;
    double min_x = 0., min_y = 0., max_x = 0., max_y = 0.;
    int x_cnt = 0, y_cnt = 0;
    m_radius = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "mesh_radius", 0, DBL_MIN, DBL_MAX, 0.); // 只针对圆形床
    if (m_radius != 0)                                                                                             // 只针对圆形床
    {
        std::string origin_str = Printer::GetInstance()->m_pconfig->GetString(section_name, "mesh_origin", "0, 0");
        strip(origin_str);
        std::istringstream iss(origin_str); // 输入流
        std::string token_str;
        while (getline(iss, token_str, ','))
        {
            m_origin.push_back(atof(token_str.c_str()));
        }
        x_cnt = y_cnt = Printer::GetInstance()->m_pconfig->GetInt(section_name, "round_probe_count", 5, 3);
        // round beds must have an odd number of points along each axis
        if ((!x_cnt) & 1)
            std::cout << "bed_mesh: probe_count must be odd for round beds" << std::endl;
        // radius may have precision to .1mm
        m_radius = floor(m_radius * 10) / 10;
        m_orig_config.radius = m_radius;
        m_orig_config.origin = m_origin;
        min_x = min_y = 0 - m_radius;
        max_x = max_y = m_radius;
    }
    else
    {
        // rectangular
        if (m_bedmesh->m_current_mesh_index && m_bedmesh->m_multi_bed_mesh_enable)
        {
            x_cnt = parse_pair(section_name, "secondary_probe_count", "3", false, 3)[0];
            y_cnt = parse_pair(section_name, "secondary_probe_count", "3", false, 3)[1];
        }
        else
        {
            x_cnt = parse_pair(section_name, "probe_count", "3", false, 3)[0];
            y_cnt = parse_pair(section_name, "probe_count", "3", false, 3)[1];
        }
        min_x = parse_pair(section_name, "mesh_min", "")[0];
        min_y = parse_pair(section_name, "mesh_min", "")[1];
        max_x = parse_pair(section_name, "mesh_max", "")[0];
        max_y = parse_pair(section_name, "mesh_max", "")[1];
        // std::cout << "x_cnt " << x_cnt << std::endl;
        // std::cout << "y_cnt " << y_cnt << std::endl;
        // std::cout << "min_x " << min_x << std::endl;
        // std::cout << "min_y " << min_y << std::endl;
        // std::cout << "max_x " << max_x << std::endl;
        // std::cout << "max_y " << max_y << std::endl;
        if (max_x <= min_x || max_y <= min_y)
            std::cout << "bed_mesh: invalid min/max points" << std::endl;
    }
    m_orig_config.x_count = m_mesh_config.x_count = x_cnt;
    m_orig_config.y_count = m_mesh_config.y_count = y_cnt;
    std::vector<double> min = {min_x, min_y};
    std::vector<double> max = {max_x, max_y};
    m_orig_config.mesh_min.assign(min.begin(), min.end());
    m_mesh_min.assign(min.begin(), min.end());
    m_orig_config.mesh_max.assign(max.begin(), max.end());
    m_mesh_max.assign(max.begin(), max.end());
    std::vector<double> pps;
    if(m_bedmesh->m_current_mesh_index && m_bedmesh->m_multi_bed_mesh_enable)
    {
        pps = parse_pair(section_name, "secondary_mesh_pps", "2", false, 0);
        m_orig_config.algo = m_mesh_config.algo = Printer::GetInstance()->m_pconfig->GetString(section_name, "secondary_algorithm", "lagrange");
        m_orig_config.tension = m_mesh_config.tension = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "secondary_bicubic_tension", 0.2, 0., 2.);
    }
    else
    {
        pps = parse_pair(section_name, "mesh_pps", "2", false, 0);
        m_orig_config.algo = m_mesh_config.algo = Printer::GetInstance()->m_pconfig->GetString(section_name, "algorithm", "lagrange");
        m_orig_config.tension = m_mesh_config.tension = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "bicubic_tension", 0.2, 0., 2.);
    }
    m_orig_config.mesh_x_pps = m_mesh_config.mesh_x_pps = (int)pps[0];
    m_orig_config.mesh_y_pps = m_mesh_config.mesh_y_pps = (int)pps[1];
    for (int i = 1; i < 100; i++)
    {
        std::string min_opt = "faulty_region_" + std::to_string(i) + "_min";
        std::string max_opt = "faulty_region_" + std::to_string(i) + "_max";
        if (Printer::GetInstance()->m_pconfig->GetString(section_name, min_opt, "") == "")
            break;
        std::vector<double> start = parse_pair(section_name, min_opt, "");
        std::vector<double> end = parse_pair(section_name, max_opt, "");
        // std::cout << "---faulty_region---" << std::endl;
        // std::cout << start[0] << "   " << start[1] << std::endl;
        // std::cout << end[0] << "   " << end[1] << std::endl;
        // Validate the corners.  If necessary reorganize them.
        // c1 = min point, c3 = max point
        //  c4 ---- c3
        //  |        |
        //  c1 ---- c2
        std::vector<double> c1 = {std::min(start[0], end[0]), std::min(start[1], end[1])};
        std::vector<double> c3 = {std::max(start[0], end[0]), std::max(start[1], end[1])};
        std::vector<double> c2 = {c1[0], c3[1]};
        std::vector<double> c4 = {c3[0], c1[1]};
        std::vector<std::vector<double>> c = {c1, c2, c3, c4};
        // Check for overlapping regions
        for (int j = 0; j < m_faulty_regions.size(); j++)
        {
            std::vector<double> prev_c1 = m_faulty_regions[j][0];
            std::vector<double> prev_c3 = m_faulty_regions[j][1];
            std::vector<double> prev_c2 = {prev_c1[0], prev_c3[1]};
            std::vector<double> prev_c4 = {prev_c3[0], prev_c1[1]};
            std::vector<std::vector<double>> prev_c = {prev_c1, prev_c2, prev_c3, prev_c4};
            // Validate that no existing corner is within the new region
            for (int k = 0; k < prev_c.size(); k++)
            {
                std::vector<double> coord = prev_c[k];
                if (within(coord, c1, c3))
                {
                    std::cout << "bed_mesh: Existing faulty_region_ overlaps added faulty_region_" << std::endl;
                }
            }
            // Validate that no new corner is within an existing region
            for (int k = 0; k < c.size(); k++)
            {
                std::vector<double> coord = c[k];
                if (within(coord, prev_c1, prev_c3))
                {
                    std::cout << "bed_mesh: Existing faulty_region_ overlaps added faulty_region_" << std::endl;
                }
            }
        }
        std::vector<std::vector<double>> faulty_regions_temp = {c1, c3};
        m_faulty_regions.push_back(faulty_regions_temp);
    }
    _verify_algorithm();
}

void BedMeshCalibrate::_verify_algorithm()
{
    struct mesh_config params = m_mesh_config;
    double x_pps = params.mesh_x_pps;
    double y_pps = params.mesh_y_pps;
    bool algo_state = false;
    for (int i = 0; i < m_ALGOS.size(); i++)
    {
        if (params.algo == m_ALGOS[i])
        {
            algo_state = true;
        }
    }
    if (!algo_state)
    {
        std::cout << "bed_mesh: Unknown algorithm" << std::endl;
    }
    // Check the algorithm against the current configuration
    double max_probe_cnt = std::max(params.x_count, params.y_count);
    double min_probe_cnt = std::min(params.x_count, params.y_count);
    if (std::max(x_pps, y_pps) == 0)
    {
        // Interpolation disabled
        m_mesh_config.algo = "direct";
    }
    else if (params.algo == "lagrange" && max_probe_cnt > 6)
    {
        // Lagrange interpolation tends to oscillate when using more than 6 samples
        std::cout << "bed_mesh: cannot exceed a probe_count of 6 when using lagrange interpolation" << std::endl;
    }
    else if (params.algo == "bicubic" && min_probe_cnt < 4)
    {
        if (max_probe_cnt > 6)
        {
            std::cout << "bed_mesh: invalid probe_count option when using bicubic \
                interpolation.  Combination of 3 points on one axis with \
                more than 6 on another is not permitted. \
                Configured PrinterProbe Count:"
                      << m_mesh_config.x_count << " " << m_mesh_config.y_count << std::endl;
        }
        else
        {
            std::cout << "bed_mesh: bicubic interpolation with a probe_count of \
                less than 4 points detected.  Forcing lagrange \
                interpolation. Configured PrinterProbe Count:"
                      << m_mesh_config.x_count << " " << m_mesh_config.y_count << std::endl;
            params.algo = "lagrange";
        }
    }
}

void BedMeshCalibrate::update_config(GCodeCommand &gcmd)
{
    // reset default configuration
    m_radius = m_orig_config.radius;
    m_origin = m_orig_config.origin;
    m_relative_reference_index = m_orig_config.rri;
    m_mesh_min = m_orig_config.mesh_min;
    m_mesh_max = m_orig_config.mesh_max;

    m_mesh_config.radius = m_orig_config.radius;
    m_mesh_config.origin = m_orig_config.origin;
    m_mesh_config.rri = m_orig_config.rri;
    m_mesh_config.x_count = m_orig_config.x_count;
    m_mesh_config.y_count = m_orig_config.y_count;
    m_mesh_config.mesh_min = m_orig_config.mesh_min;
    m_mesh_config.mesh_max = m_orig_config.mesh_max;
    m_mesh_config.mesh_x_pps = m_orig_config.mesh_x_pps;
    m_mesh_config.mesh_y_pps = m_orig_config.mesh_y_pps;
    m_mesh_config.algo = m_orig_config.algo;
    m_mesh_config.tension = m_orig_config.tension;

    std::map<std::string, std::string> params;
    gcmd.get_command_parameters(params);
    bool need_cfg_update = false;
    if (params.find("RELATIVE_REFERENCE_INDEX") != params.end())
    {
        m_relative_reference_index = gcmd.get_int("RELATIVE_REFERENCE_INDEX", INT32_MIN);
        if (m_relative_reference_index < 0)
        {
            m_relative_reference_index = INT32_MIN;
        }
        need_cfg_update = true;
    }
    if (m_radius != 0.) // 圆形
    {
        if (params.find("MESH_RADIUS") != params.end())
        {
            m_radius = gcmd.get_double("MESH_RADIUS", DBL_MIN);
            m_radius = floor(m_radius * 10) / 10;
            m_mesh_min[0] = -m_radius;
            m_mesh_min[1] = -m_radius;
            m_mesh_max[0] = m_radius;
            m_mesh_max[1] = m_radius;
            need_cfg_update = true;
        }
        if (params.find("MESH_ORIGIN") != params.end())
        {
            // m_origin = parse_pair(gcmd, ("MESH_ORIGIN",)) //---??---bed_mesh
            need_cfg_update = true;
        }
        if (params.find("ROUND_PROBE_COUNT") != params.end())
        {
            int cnt = gcmd.get_int("ROUND_PROBE_COUNT", INT32_MIN, 3);
            m_mesh_config.x_count = cnt;
            m_mesh_config.y_count = cnt;
            need_cfg_update = true;
        }
    }
    else
    {
        if (params.find("MESH_MIN") != params.end())
        {
            // m_mesh_min = parse_pair(gcmd, ("MESH_MIN",)) //---??---bed_mesh
            need_cfg_update = true;
        }
        if (params.find("MESH_MAX") != params.end())
        {
            // m_mesh_max = parse_pair(gcmd, ("MESH_MAX",)) //---??---bed_mesh
            need_cfg_update = true;
        }
        if (params.find("PROBE_COUNT") != params.end())
        {
            // x_cnt, y_cnt = parse_pair(gcmd, ("PROBE_COUNT",), check=False, cast=int, minval=3)  //---??---bed_mesh
            // m_mesh_config.x_count = x_cnt;
            // m_mesh_config.y_count = y_cnt;
            need_cfg_update = true;
        }
    }
    if (params.find("ALGORITHM") != params.end())
    {
        m_mesh_config.algo = gcmd.get_string("ALGORITHM", "");
        need_cfg_update = true;
    }

    if (need_cfg_update)
    {
        _verify_algorithm();
        _generate_points();
        printf("Generating new points...\n");
        print_generated_points();
        std::vector<std::vector<double>> pts = _get_adjusted_points();
        m_probe_helper->update_probe_points(pts, 3);
    }
    else
    {
        m_points = m_orig_points;
        std::vector<std::vector<double>> pts = _get_adjusted_points();
        // std::cout << "no update cfg" << std::endl;
        // for(auto pt : pts)
        // {
        //     std::cout << pt[0] << "  " << pt[1] << std::endl;
        // }
        m_probe_helper->update_probe_points(pts, 3);
    }
}

std::vector<std::vector<double>> BedMeshCalibrate::_get_adjusted_points()
{
    if (m_substituted_indices.size() == 0)
        return m_points;
    std::vector<std::vector<double>> adj_pts;
    int last_index = 0;
    for (auto index : m_substituted_indices)
    {
        adj_pts.insert(adj_pts.end(), m_points.begin() + last_index, m_points.begin() + index.first);
        adj_pts.insert(adj_pts.end(), index.second.begin(), index.second.end());
        // Add one to the last index to skip the point we are replacing
        last_index = index.first + 1;
    }
    adj_pts.insert(adj_pts.end(), m_points.begin() + last_index, m_points.end());
    return adj_pts;
}

void BedMeshCalibrate::cmd_BED_MESH_CALIBRATE(GCodeCommand &gcmd)
{   
    Printer::GetInstance()->m_gcode_io->single_command("BED_MESH_SET_INDEX INDEX=0");
    if (gcmd.get_string("METHOD", "automatic") == "fast") { 
        ZMesh *z_mesh = m_bedmesh->get_mesh();
        if (z_mesh != nullptr) {
            // 存储上一次结果值 
            std::vector<std::vector<double>> probed_matrix = z_mesh->get_probed_matrix();
            if (probed_matrix.size() > 0) {
                std::cout << " last auto leveling result: "<< std::endl;
                for (int i=0; i<probed_matrix.size();i++)
                {
                    for (int j=0; j<probed_matrix.at(i).size();j++)
                    {
                        std::cout << probed_matrix.at(i).at(j) << " ";
                    }
                    std::cout <<  "\n"<< std::endl;
                }
                if (probed_matrix.at(0).size() % 2 == 0)
                {
                    m_probe_helper->m_last_fast_probe_points_z.clear();
                    m_probe_helper->m_last_fast_probe_points_z.push_back(probed_matrix.at(0).at(0));
                    m_probe_helper->m_last_fast_probe_points_z.push_back(probed_matrix.at(0).at(probed_matrix.at(0).size()-1));
                    m_probe_helper->m_last_fast_probe_points_z.push_back(probed_matrix.at(probed_matrix.size()-1).at(probed_matrix.at(probed_matrix.size()-1).size()-1));
                    m_probe_helper->m_last_fast_probe_points_z.push_back(probed_matrix.at(probed_matrix.size()-1).at(0));

                    m_probe_helper->m_fast_probe_points.clear();
                    m_probe_helper->m_fast_probe_points.push_back(m_probe_helper->m_probe_points.at(0));
                    m_probe_helper->m_fast_probe_points.push_back(m_probe_helper->m_probe_points.at(probed_matrix.at(0).size()-1));
                    m_probe_helper->m_fast_probe_points.push_back(m_probe_helper->m_probe_points.at(m_probe_helper->m_probe_points.size()-probed_matrix.at(0).size()));
                    m_probe_helper->m_fast_probe_points.push_back(m_probe_helper->m_probe_points.at(m_probe_helper->m_probe_points.size()-1));
                }
                else
                {
                    m_probe_helper->m_last_fast_probe_points_z.clear();
                    m_probe_helper->m_last_fast_probe_points_z.push_back(probed_matrix.at(0).at(0));
                    m_probe_helper->m_last_fast_probe_points_z.push_back(probed_matrix.at(0).at(probed_matrix.at(0).size()-1));
                    m_probe_helper->m_last_fast_probe_points_z.push_back(probed_matrix.at(probed_matrix.size()-1).at(probed_matrix.at(probed_matrix.size()-1).size()-1));
                    m_probe_helper->m_last_fast_probe_points_z.push_back(probed_matrix.at(probed_matrix.size()-1).at(0));

                    m_probe_helper->m_fast_probe_points.clear();
                    m_probe_helper->m_fast_probe_points.push_back(m_probe_helper->m_probe_points.at(0));
                    m_probe_helper->m_fast_probe_points.push_back(m_probe_helper->m_probe_points.at(probed_matrix.at(0).size()-1));
                    m_probe_helper->m_fast_probe_points.push_back(m_probe_helper->m_probe_points.at(m_probe_helper->m_probe_points.size()-1));
                    m_probe_helper->m_fast_probe_points.push_back(m_probe_helper->m_probe_points.at(m_probe_helper->m_probe_points.size()-probed_matrix.at(0).size()));
                }
                std::cout << " last auto leveling fast point result: "<< std::endl;
                for (int k=0;k<m_probe_helper->m_last_fast_probe_points_z.size();k++){
                    std::cout << k  << "(" << m_probe_helper->m_fast_probe_points.at(k).at(0) << ","<< m_probe_helper->m_fast_probe_points.at(k).at(1) << "):" << m_probe_helper->m_last_fast_probe_points_z.at(k)<< std::endl;
                }
            }
        } else {
            m_probe_helper->m_fast_probe_points.clear();
            m_probe_helper->m_last_fast_probe_points_z.clear();
            std::cout << " last auto leveling result is null, continue auto leveling step....................................."<< std::endl;
        }
    }
    // m_bedmesh->set_mesh(nullptr);
    // // TODO: m_bedmesh delete
    // delete m_bedmesh->get_mesh(); 
    update_config(gcmd);
    m_probe_helper->start_probe(gcmd);
    Printer::GetInstance()->m_auto_leveling->m_enable_fast_probe = false;
}
std::string BedMeshCalibrate::probe_finalize(std::vector<double> offsets, std::vector<std::vector<double>> positions)
{
    // std::vector<double> offset_points = parse_pair("bed_mesh", "offset_points", "");
    // if (offset_points.size() != positions.size())
    // {
    //     LOG_E("offset_points size error! size = %d\n", offset_points.size());
    // }
    // else
    // {
    //     for (int i = 0; i < positions.size(); i++)
    //     {
    //         positions[i][2] += offset_points[i];
    //     }
    // }
    double x_offset = offsets[0];
    double y_offset = offsets[1];
    double z_offset = offsets[2];
    std::vector<double> x_positions;
    std::vector<double> y_positions;
    for (int i = 0; i < positions.size(); i++)
    {
        x_positions.push_back(positions[i][0]);
        y_positions.push_back(positions[i][1]);
    }
    m_mesh_config.min_x = *min_element(x_positions.begin(), x_positions.end()) + x_offset;
    m_mesh_config.max_x = *max_element(x_positions.begin(), x_positions.end()) + x_offset;
    m_mesh_config.min_y = *min_element(y_positions.begin(), y_positions.end()) + y_offset;
    m_mesh_config.max_y = *max_element(y_positions.begin(), y_positions.end()) + y_offset;
    double x_cnt = m_mesh_config.x_count;
    double y_cnt = m_mesh_config.y_count;
    if (m_substituted_indices.size() != 0)
    {
        // Replace substituted points with the original generated point.  Its Z Value is the average probed Z of the substituted points.
        std::vector<std::vector<double>> corrected_pts;
        int idx_offset = 0;
        int start_idx = 0;
        for (auto index : m_substituted_indices)
        {
            std::vector<std::vector<double>> pts = index.second;
            std::vector<double> fpt = {m_points[index.first][0] - offsets[0], m_points[index.first][1] - offsets[1]};
            // offset the index to account for additional samples
            int idx = index.first + idx_offset;
            // Add "normal" points
            corrected_pts.insert(corrected_pts.end(), positions.begin() + start_idx, positions.begin() + idx);
            std::vector<double> z_points;
            for (int j = idx; j < idx + pts.size(); j++)
            {
                z_points.push_back(positions[j][2]);
            }
            double avg_z = accumulate(z_points.begin(), z_points.end(), 0.) / pts.size();
            idx_offset += pts.size() - 1;
            start_idx = idx + pts.size();
            fpt.push_back(avg_z);
            corrected_pts.push_back(fpt);
        }
        corrected_pts.insert(corrected_pts.end(), positions.begin() + start_idx, positions.end());
        // validate corrected positions
        if (m_points.size() != corrected_pts.size())
        {
            _dump_points(positions, corrected_pts, offsets);
            std::cout << "bed_mesh: invalid position list size, generated count: " << m_points.size() << " probed count: " << corrected_pts.size() << std::endl;
        }
        for (int i = 0; i < m_points.size(); i++)
        {
            std::vector<double> gen_pt = m_points[i];
            std::vector<double> probed = corrected_pts[i];
            std::vector<double> off_pt = {gen_pt[0] - offsets[0], gen_pt[1] - offsets[1]};
            if (!isclose(off_pt[0], probed[0], 0.1) || !isclose(off_pt[1], probed[1], 0.1))
            {
                _dump_points(positions, corrected_pts, offsets);
                std::cout << "bed_mesh: point mismatch" << std::endl;
            }
        }
        positions = corrected_pts;
    }
    if (m_relative_reference_index)
    {
        // zero out probe z offset and set offset relative to reference index
        z_offset = positions[m_relative_reference_index][2];
    }
    std::vector<std::vector<double>> probed_matrix;
    std::vector<double> row;
    std::vector<double> prev_pos = positions[0];
    for (int i = 0; i < positions.size(); i++)
    {
        std::vector<double> pos = positions[i];
        if (!isclose(pos[1], prev_pos[1], 0.01))
        {
            // y has changed, append row and start new
            probed_matrix.push_back(row);
            std::vector<double>().swap(row);
        }
        if (pos[0] > prev_pos[0])
        {
            // probed in the positive direction
            row.push_back(pos[2] - z_offset);
        }
        else
        {
            // probed in the negative direction
            row.insert(row.begin(), pos[2] - z_offset);
        }
        prev_pos = pos;
    }
    // append last row
    probed_matrix.push_back(row);
    // make sure the y-axis is the correct length
    if (probed_matrix.size() != y_cnt)
    {
        printf("bed_mesh: Invalid y-axis table length\nProbed table length: %d Probed Table:\n%s", probed_matrix.size(), "probed_matrix\n");
    }
    if (m_radius != 0)
    {
        // round bed, extrapolate probed values to create a square mesh
        for (int i = 0; i < probed_matrix.size(); i++)
        {
            std::vector<double> row = probed_matrix[i];
            int row_size = row.size();
            if (!row_size & 1)
            {
                // an even number of points in a row shouldn't be possible
                std::string msg = "bed_mesh: incorrect number of points sampled on X\n";
                msg += "Probed Table:\n";
                msg += "probed_matrix";
                std::cout << msg << std::endl;
            }
            int buf_cnt = (x_cnt - row_size); // 2
            if (buf_cnt == 0)
                continue;
            double left_buffer = row[0] * buf_cnt;
            double right_buffer = row[row_size - 1] * buf_cnt;
            row[0] = left_buffer;
            row.insert(row.end(), right_buffer);
        }
    }
    //  make sure that the x-axis is the correct length
    for (int i = 0; i < probed_matrix.size(); i++)
    {
        std::vector<double> row = probed_matrix[i];
        // for (int j = 0; j < row.size(); j++)
        // {
        //     LOG_D("%f ", row[j]);
        // }
        // LOG_D("\n");
        if (row.size() != x_cnt)
        {
            printf("bed_mesh: invalid x-axis table length\nProbed table length: %d Probed Table:\n%s", probed_matrix.size(), "probed_matrix");
            return "";
        }
    }
    if (m_z_mesh)
    {
        delete m_z_mesh;
    }
    m_z_mesh = new ZMesh(m_mesh_config); // 注意
    m_z_mesh->build_mesh(probed_matrix);
    m_bedmesh->set_mesh(m_z_mesh);
    printf("Bed Mesh Leveling Complete\n");
    if (m_bedmesh->m_current_mesh_index)
    {
        m_bedmesh->m_pmgr->save_profile(to_string(m_bedmesh->m_current_mesh_index));
    }
    else
    {
        m_bedmesh->m_pmgr->save_profile(PROFILE_PREFIX_SUFFIX_NAME);
    }
    return "";
}

void BedMeshCalibrate::_dump_points(std::vector<std::vector<double>> probed_pts, std::vector<std::vector<double>> corrected_pts, std::vector<double> offsets)
{
    // logs generated points with offset applied, points received
    // from the finalize callback, and the list of corrected points
    int max_len = std::max(std::max(m_points.size(), probed_pts.size()), corrected_pts.size());
    for (int i = 0; i < max_len; i++)
    {
        std::vector<double> gen_pt;
        std::vector<double> probed_pt;
        std::vector<double> corr_pt;
        if (i < m_points.size())
        {
            gen_pt.push_back(m_points[i][0] - offsets[0]);
            gen_pt.push_back(m_points[i][1] - offsets[0]);
        }
        if (i < probed_pts.size())
        {
            probed_pt = probed_pts[i];
        }
        if (i < corrected_pts.size())
        {
            corr_pt = corrected_pts[i];
        }
    }
}

MoveSplitter::MoveSplitter(std::string section_name)
{
    m_split_delta_z = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "split_delta_z", .025, 0.01);
    m_move_check_distance = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "move_check_distance", 5., 3.);
    double fade_offset = 0.;
}

MoveSplitter::~MoveSplitter()
{
}

void MoveSplitter::initialize(ZMesh *mesh, double fade_offset)
{
    m_z_mesh = mesh;
    m_fade_offset = fade_offset;
}
// 这个Python原函数build_move用于构建一个运动轨迹，它需要三个参数，分别是上一个位置prev_pos、下一个位置next_pos和一个缩放因子factor。
// 该函数的主要功能是计算轨迹的总运动距离、每个轴的运动方向以及轨迹开始时的偏移量等信息，并将这些信息存储在对象的属性中。
void MoveSplitter::build_move(std::vector<double> prev_pos, std::vector<double> next_pos, double factor)
{
    m_prev_pos = prev_pos;
    m_next_pos = next_pos;
    m_current_pos = prev_pos;
    m_z_factor = factor;
    m_z_offset = _calc_z_offset(prev_pos);
    // std::cout << "m_z_offset:" << m_z_offset << std::endl;
    m_traverse_complete = false;
    m_distance_checked = 0.;
    std::vector<double> axes_d;
    for (int i = 0; i < 4; i++)
    {
        axes_d.push_back(m_next_pos[i] - m_prev_pos[i]);
    }
    double sum = 0.;
    for (int i = 0; i < 3; i++)
    {
        sum += axes_d[i] * axes_d[i];
    }
    m_total_move_length = sqrt(sum);
    for (int i = 0; i < axes_d.size(); i++)
    {
        if (!isclose(axes_d[i], 0.0f, 1e-10)) // 距离差值不为0，即有运动
        {
            m_axis_move[i] = true;
        }
        else
        {
            m_axis_move[i] = false;
        }
    }
}

double MoveSplitter::_calc_z_offset(std::vector<double> pos)
{
    double z = m_z_mesh->calc_z(pos[0], pos[1]);
    double offset = m_fade_offset;
    return m_z_factor * (z - offset) + offset;
}
void MoveSplitter::_set_next_move(double distance_from_prev)
{
    double t = distance_from_prev / m_total_move_length;
    if (t > 1. || t < 0.)
    {
        printf("bed_mesh: Slice distance is negative or greater than entire move length");
    }
    for (int i = 0; i < 4; i++)
    {
        if (m_axis_move[i])
        {
            m_current_pos[i] = lerp(t, m_prev_pos[i], m_next_pos[i]);
        }
    }
}

std::vector<double> MoveSplitter::split()
{
    std::vector<double> ret = {};
    if (m_traverse_complete == false)
    {
        if (m_axis_move[0] || m_axis_move[1])
        {
            // X and/or Y axis move, traverse if necessary
            while (m_distance_checked + m_move_check_distance < m_total_move_length)
            {
                m_distance_checked += m_move_check_distance;
                _set_next_move(m_distance_checked);
                double next_z = _calc_z_offset(m_current_pos);
                if (fabs(next_z - m_z_offset) >= m_split_delta_z)
                {
                    m_z_offset = next_z;
                    ret.push_back(m_current_pos[0]);
                    ret.push_back(m_current_pos[1]);
                    ret.push_back(m_current_pos[2] + m_z_offset);
                    ret.push_back(m_current_pos[3]);
                    return ret;
                }
            }
        }
        // end of move reached
        m_current_pos = m_next_pos;
        m_z_offset = _calc_z_offset(m_current_pos);
        // Its okay to add Z-Offset to the final move, since it will not be
        // used again.
        m_current_pos[2] += m_z_offset;
        m_traverse_complete = true;
        return m_current_pos;
    }
    else
    {
        // Traverse complete
        return ret;
    }
}

ZMesh::ZMesh(struct mesh_config params)
{
    m_mesh_params = params;
    m_avg_z = 0.;
    m_mesh_offsets = {0., 0.};
    m_mesh_x_min = params.min_x;
    m_mesh_x_max = params.max_x;
    m_mesh_y_min = params.min_y;
    m_mesh_y_max = params.max_y;
    // Set the interpolation algorithm
    m_interpolation_algos["lagrange"] = std::bind(&ZMesh::_sample_lagrange, this, std::placeholders::_1);
    m_interpolation_algos["bicubic"] = std::bind(&ZMesh::_sample_bicubic, this, std::placeholders::_1);
    m_interpolation_algos["direct"] = std::bind(&ZMesh::_sample_direct, this, std::placeholders::_1);
    m_sample = m_interpolation_algos[params.algo];
    // Number of points to interpolate per segment
    int mesh_x_pps = params.mesh_x_pps;
    int mesh_y_pps = params.mesh_y_pps;
    int px_cnt = params.x_count;
    int py_cnt = params.y_count;
    m_mesh_x_count = (px_cnt - 1) * mesh_x_pps + px_cnt;
    m_mesh_y_count = (py_cnt - 1) * mesh_y_pps + py_cnt;
    m_x_mult = mesh_x_pps + 1;
    m_y_mult = mesh_y_pps + 1;
    m_mesh_x_dist = (m_mesh_x_max - m_mesh_x_min) / (m_mesh_x_count - 1);
    m_mesh_y_dist = (m_mesh_y_max - m_mesh_y_min) / (m_mesh_y_count - 1);
}
ZMesh::~ZMesh()
{
}

std::vector<std::vector<double>> ZMesh::get_mesh_matrix()
{
    std::vector<std::vector<double>> result = {{}};
    for (const auto &line : m_mesh_matrix)
    {
        std::vector<double> row;
        for (const auto &z : line)
        {
            row.push_back(round(z * 10e6) / 10e6);
        }
        result.push_back(row);
    }
    return result;
}

std::vector<std::vector<double>> ZMesh::get_probed_matrix()
{
    std::vector<std::vector<double>> ret;
    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(6);
    for (int i = 0; i < m_probed_matrix.size(); i++)
    {
        std::vector<double> line;
        for (int j = 0; j < m_probed_matrix[i].size(); j++)
        {
            double value = round(m_probed_matrix[i][j] * 10e6);
            value /= 10e6;
            line.push_back(value);
        }
        ret.push_back(line);
    }
    if (m_probed_matrix.size() != 0)
    {
        return ret;
    }
    return ret;
}

struct mesh_config ZMesh::get_mesh_params()
{
    return m_mesh_params;
}

void ZMesh::print_probed_matrix(std::function<void(std::string, bool)> print_func)
{
    if (m_probed_matrix.size() != 0)
    {
        std::string msg = "Mesh Leveling Probed Z positions:\n";
        for (int i = 0; i < m_probed_matrix.size(); i++)
        {
            for (int j = 0; j < m_probed_matrix[i].size(); j++)
            {
                msg += std::to_string(m_probed_matrix[i][j]);
                msg += "\t";
            }
            msg += "\n";
        }
        LOG_I("%s\n", msg.c_str());
    }
    else
    {
        printf("bed_mesh: bed has not been probed\n");
    }
}

void ZMesh::print_mesh(std::function<void(std::string, bool)> print_func, double move_z)
{
    std::vector<std::vector<double>> matrix = get_mesh_matrix();
    if (matrix.size() != 0)
    {
        std::string msg = "Mesh X,Y: " + std::to_string(m_mesh_x_count) + "," + std::to_string(m_mesh_y_count) + "\n";
        if (move_z != 0)
        {
            msg += "Search Height: " + std::to_string(move_z);
            msg += "\n";
        }
        msg = msg + "Mesh Offsets: " + "X=" + std::to_string(m_mesh_offsets[0]) + " " + "Y=" + std::to_string(m_mesh_offsets[1]) + "\n";
        msg = msg + "Mesh Average: " + std::to_string(m_avg_z) + "\n";
        std::vector<double> rng = get_z_range();
        msg = msg + "Mesh Range: " + "min=" + std::to_string(rng[0]) + " " + "max=" + std::to_string(rng[1]) + "\n";
        msg = msg + "Interpolation Algorithm: " + m_mesh_params.algo + "\n";
        msg = msg + "Measured points:\n";
        for (int i = 0; i < m_probed_matrix.size(); i++) //
        {
            for (int j = 0; j < m_probed_matrix[i].size(); j++)
            {
                msg += std::to_string(m_probed_matrix[i][j]);
                msg += "\t";
            }
            msg += "\n";
        }
        msg += "\n";
        LOG_I("%s", msg.c_str());
    }
    else
    {
        printf("Z Mesh not generated\n");
    }
}

void ZMesh::build_mesh(std::vector<std::vector<double>> z_matrix)
{
    m_probed_matrix = z_matrix;
    m_sample(z_matrix);

    double sum_mesh = 0.0;
    int count_mesh = 0;
    for (const auto &row : m_mesh_matrix)
    {
        for (const auto &val : row)
        {
            sum_mesh += val;
            count_mesh++;
        }
    }
    m_avg_z = sum_mesh / count_mesh;

    // 在计算平均值后，我们将其乘以 100 并使用 round() 函数将其四舍五入到最接近的百分之一。
    // 最后，我们将平均值除以 100 来将其转换回原始比例。注意，在 C++ 中，round() 函数返回最接近给定浮点数的整数，
    // 因此我们将其乘以 100 来将其舍入到最接近的百分之一，然后再将其除以 100 来将其转换回原始比例。
    m_avg_z = round(m_avg_z * 100.0) / 100.0;

    // print_mesh(NULL, 0);
}

void ZMesh::set_mesh_offsets(std::vector<double> offsets)
{
    for (int i = 0; i < offsets.size(); i++)
    {
        if (offsets[i] != DBL_MIN)
        {
            m_mesh_offsets[i] = offsets[i];
        }
    }
}
double ZMesh::get_x_coordinate(int index)
{
    return m_mesh_x_min + m_mesh_x_dist * index;
}
double ZMesh::get_y_coordinate(int index)
{
    return m_mesh_y_min + m_mesh_y_dist * index;
}

double ZMesh::calc_z(double x, double y)
{
    if (m_mesh_matrix.size() != 0)
    {
        std::vector<std::vector<double>> tbl = m_mesh_matrix;
        double tx = _get_linear_index(x + m_mesh_offsets[0], 0)[0];
        int xidx = (int)_get_linear_index(x + m_mesh_offsets[0], 0)[1];
        double ty = _get_linear_index(y + m_mesh_offsets[1], 1)[0];
        int yidx = (int)_get_linear_index(y + m_mesh_offsets[1], 1)[1];
        double z0 = lerp(tx, tbl[yidx][xidx], tbl[yidx][xidx + 1]);
        double z1 = lerp(tx, tbl[yidx + 1][xidx], tbl[yidx + 1][xidx + 1]);
        return lerp(ty, z0, z1);
    }
    else
    {
        // No mesh table generated, no z-adjustment
        return 0.;
    }
}

std::vector<double> ZMesh::get_z_range()
{
    std::vector<double> ret_min_max;
    if (m_mesh_matrix.size() != 0)
    {
        double mesh_min = std::numeric_limits<double>::max();
        double mesh_max = std::numeric_limits<double>::min();
        for (int i = 0; i < m_mesh_matrix.size(); i++)
        {
            double row_min = *std::min_element(m_mesh_matrix[i].begin(), m_mesh_matrix[i].end());
            double row_max = *std::max_element(m_mesh_matrix[i].begin(), m_mesh_matrix[i].end());
            if (row_min < mesh_min)
            {
                mesh_min = row_min;
            }
            if (row_max > mesh_max)
            {
                mesh_max = row_max;
            }
        }
        ret_min_max.push_back(mesh_min);
        ret_min_max.push_back(mesh_max);
        return ret_min_max;
    }
    else
    {
        ret_min_max.push_back(0.);
        ret_min_max.push_back(0.);
        return ret_min_max;
    }
}

std::vector<double> ZMesh::_get_linear_index(double coord, int axis)
{
    double mesh_min = 0.;
    double mesh_cnt = 0.;
    double mesh_dist = 0.;
    std::function<double(int)> cfunc;
    if (axis == 0)
    {
        // X-axis
        mesh_min = m_mesh_x_min;
        mesh_cnt = m_mesh_x_count;
        mesh_dist = m_mesh_x_dist;
        cfunc = std::bind(&ZMesh::get_x_coordinate, this, std::placeholders::_1);
    }
    else
    {
        // Y-axis
        mesh_min = m_mesh_y_min;
        mesh_cnt = m_mesh_y_count;
        mesh_dist = m_mesh_y_dist;
        cfunc = std::bind(&ZMesh::get_y_coordinate, this, std::placeholders::_1);
    }
    double t = 0.;
    double idx = (int)(floor((coord - mesh_min) / mesh_dist));
    idx = constrain(idx, 0, mesh_cnt - 2);
    t = (coord - cfunc(idx)) / mesh_dist;
    std::vector<double> ret_t_idx;
    t = constrain(t, 0., 1.);
    ret_t_idx.push_back(t);
    ret_t_idx.push_back(idx);
    return ret_t_idx;
}
void ZMesh::_sample_direct(std::vector<std::vector<double>> z_matrix)
{
    m_mesh_matrix = z_matrix;
}
/**
 * @brief 用于使用拉格朗日插值法对给定的Z矩阵进行插值。
 *
 * @param z_matrix 二维数组，表示要进行插值的原始数据
 */
void ZMesh::_sample_lagrange(std::vector<std::vector<double>> z_matrix)
{
    int x_mult = m_x_mult; // 获取探测点数
    int y_mult = m_y_mult;
    m_mesh_matrix.resize(m_mesh_x_count, std::vector<double>(m_mesh_y_count, 0.0));
    // 把准备插值的位置的值设置为0，其他的值保持不变
    for (int j = 0; j < this->m_mesh_y_count; j++)
    {
        for (int i = 0; i < this->m_mesh_x_count; i++)
        {
            if ((i % x_mult) || (j % y_mult))
            {
                continue;
            }
            else
            {
                this->m_mesh_matrix[j][i] = z_matrix[j / y_mult][i / x_mult];
            }
        }
    }
    std::vector<std::vector<double>> ret = _get_lagrange_coords(); // 获取拉格朗日插值的坐标点
    std::vector<double> xpts = ret[0];
    std::vector<double> ypts = ret[1];
    double x = 0., y = 0.;
    // Interpolate X coordinates
    for (int i = 0; i < m_mesh_y_count; i++)
    {
        // only interpolate X-rows that have probed coordinates
        if (i % y_mult != 0)
            continue;
        for (int j = 0; j < m_mesh_x_count; j++) // 只对非探测点进行插值
        {
            if (j % x_mult == 0)
            {
                continue;
            }
            x = get_x_coordinate(j);
            m_mesh_matrix[i][j] = _calc_lagrange(xpts, x, i, 0);
        }
    }

    // Interpolate Y coordinates
    for (int i = 0; i < m_mesh_x_count; i++)
    {
        for (int j = 0; j < m_mesh_y_count; j++)
        {
            if (j % y_mult == 0)
            {
                continue;
            }
            y = get_y_coordinate(j);
            m_mesh_matrix[j][i] = _calc_lagrange(ypts, y, i, 1);
        }
    }
#if BED_MESH_DEBUG == 1
    printf("插值后的矩阵数据为：\n");
    for (int i = 0; i < m_mesh_matrix.size(); i++)
    {
        for (int j = 0; j < m_mesh_matrix[i].size(); j++)
        {
            printf("%f\t", m_mesh_matrix[i][j]);
        }
        printf("\n");
    }
#endif
}
/**
 * @brief 用于生成网格矩阵中每个节点的拉格朗日坐标。
 *
 * @return std::vector<std::vector<double>>
 */
std::vector<std::vector<double>> ZMesh::_get_lagrange_coords()
{
    std::vector<double> xpts;
    std::vector<double> ypts;
    for (int i = 0; i < m_mesh_params.x_count; i++)
    {
        xpts.push_back(get_x_coordinate(i * m_x_mult));
    }
    for (int i = 0; i < m_mesh_params.y_count; i++)
    {
        ypts.push_back(get_y_coordinate(i * m_y_mult));
    }
    std::vector<std::vector<double>> ret_x_y;
    ret_x_y.push_back(xpts);
    ret_x_y.push_back(ypts);
    return ret_x_y;
}

double ZMesh::_calc_lagrange(std::vector<double> lpts, double c, int vec, int axis)
{
    int pt_cnt = lpts.size();
    double total = 0.0f;
    for (int i = 0; i < pt_cnt; i++)
    {
        double n = 1.0f;
        double d = 1.0f;
        double z = 0.0f;
        for (int j = 0; j < pt_cnt; j++)
        {
            if (j == i)
            {
                continue;
            }
            n *= (c - lpts[j]);
            d *= (lpts[i] - lpts[j]);
        }
        if (axis == 0)
        {
            // Calc X-Axis
            z = m_mesh_matrix[vec][i * m_x_mult];
        }
        else
        {
            // Calc Y-Axis
            z = m_mesh_matrix[i * m_y_mult][vec];
        }
        total += z * n / d;
    }
    return total;
}

void ZMesh::_sample_bicubic(std::vector<std::vector<double>> z_matrix)
{
    // should work for any number of probe points above 3x3
    int x_mult = m_x_mult;
    int y_mult = m_y_mult;
    double c = m_mesh_params.tension;
    m_mesh_matrix.resize(m_mesh_x_count, std::vector<double>(m_mesh_y_count, 0.0));
    for (int j = 0; j < m_mesh_y_count; j++)
    {
        for (int i = 0; i < m_mesh_x_count; i++)
        {
            if ((i % x_mult) || (j % y_mult))
            {
                m_mesh_matrix[j][i] = 0;
            }
            else
            {
                m_mesh_matrix[j][i] = z_matrix[j / y_mult][i / x_mult];
            }
        }
    }
    // Interpolate X values
    for (int y = 0; y < m_mesh_y_count; y++)
    {
        if (y % y_mult != 0)
        {
            continue;
        }
        for (int x = 0; x < m_mesh_x_count; x++)
        {
            if (x % x_mult == 0)
            {
                continue;
            }
            std::vector<double> pts = _get_x_ctl_pts(x, y);
            m_mesh_matrix[y][x] = _cardinal_spline(pts, c);
        }
    }
    // Interpolate Y values
    std::vector<double> pts;
    for (int x = 0; x < m_mesh_x_count; x++)
    {
        for (int y = 0; y < m_mesh_y_count; y++)
        {
            if (y % y_mult == 0)
            {
                continue;
            }
            pts = _get_y_ctl_pts(x, y);
            m_mesh_matrix[y][x] = _cardinal_spline(pts, c);
        }
    }
}

std::vector<double> ZMesh::_get_x_ctl_pts(double x, double y)
{
    // Fetch control points and t for a X value in the mesh
    double x_mult = m_x_mult;
    std::vector<double> x_row = m_mesh_matrix[y];
    int last_pt = m_mesh_x_count - 1 - x_mult;
    double p0 = 0., p1 = 0., p2 = 0., p3 = 0., t = 0.;
    bool found = false;
    if (x < x_mult)
    {
        p0 = p1 = x_row[0];
        p2 = x_row[x_mult];
        p3 = x_row[2 * x_mult];
        t = x / float(x_mult);
    }
    else if (x > last_pt)
    {
        p0 = x_row[last_pt - x_mult];
        p1 = x_row[last_pt];
        p2 = p3 = x_row[last_pt + x_mult];
        t = (x - last_pt) / float(x_mult);
    }
    else
    {
        bool found = false;
        for (int i = x_mult; i < last_pt; i += x_mult)
        {
            if (x > i && x < (i + x_mult))
            {
                p0 = x_row[i - x_mult];
                p1 = x_row[i];
                p2 = x_row[i + x_mult];
                p3 = x_row[i + 2 * x_mult];
                t = (x - i) / float(x_mult);
                found = true;
                break;
            }
        }

        if (!found)
        {
            printf("bed_mesh: Error finding x control points\n");
        }
    }
    std::vector<double> ret = {p0, p1, p2, p3, t};
    return ret;
}
std::vector<double> ZMesh::_get_y_ctl_pts(double x, double y)
{
    // Fetch control points and t for a Y value in the mesh
    double y_mult = m_y_mult;
    int last_pt = m_mesh_y_count - 1 - y_mult;
    std::vector<std::vector<double>> y_col = m_mesh_matrix;
    double p0 = 0., p1 = 0., p2 = 0., p3 = 0., t = 0.;
    bool found = false;
    if (y < y_mult)
    {
        p0 = p1 = y_col[0][x];
        p2 = y_col[y_mult][x];
        p3 = y_col[2 * y_mult][x];
        t = y / float(y_mult);
    }
    else if (y > last_pt)
    {
        p0 = y_col[last_pt - y_mult][x];
        p1 = y_col[last_pt][x];
        p2 = p3 = y_col[last_pt + y_mult][x];
        t = (y - last_pt) / float(y_mult);
    }
    else
    {
        found = false;
        for (int i = y_mult; i < last_pt; i += y_mult)
        {
            if (y > i && y < (i + y_mult))
            {
                p0 = y_col[i - y_mult][x];
                p1 = y_col[i][x];
                p2 = y_col[i + y_mult][x];
                p3 = y_col[i + 2 * y_mult][x];
                t = (y - i) / float(y_mult);
                found = true;
                break;
            }
        }
        if (!found)
        {
            printf("bed_mesh: Error finding y control points\n");
        }
    }
    std::vector<double> ret = {p0, p1, p2, p3, t};
    return ret;
}

double ZMesh::_cardinal_spline(std::vector<double> p, double tension)
{
    double t = p[4];
    double t2 = t * t;
    double t3 = t2 * t;
    double m1 = tension * (p[2] - p[0]);
    double m2 = tension * (p[3] - p[1]);
    double a = p[1] * (2 * t3 - 3 * t2 + 1);
    double b = p[2] * (-2 * t3 + 3 * t2);
    double c = m1 * (t3 - 2 * t2 + t);
    double d = m2 * (t3 - t2);
    return a + b + c + d;
}
/**
 * @brief 保存文件管理部分和源码相比改动较大
 *
 * @param section_name
 * @param bedmesh
 */
ProfileManager::ProfileManager(std::string section_name, BedMesh *bedmesh)
{
    m_bedmesh = bedmesh;
    m_current_profile = "";
    m_z_mesh = nullptr;
    load_profile(PROFILE_PREFIX_SUFFIX_NAME); // 里面会判断是否有default，没有就不加载
    // m_profiles = {0};
    // m_current_profile = "";
    // m_stored_profs = Printer::GetInstance()->m_pconfig->get_prefix_sections(section_name);
    // for (const std::string &str : m_stored_profs)
    // {
    //     std::cout << " m_stored_profs " << str << " ";
    // }
    // std::cout << std::endl;
    // m_version = Printer::GetInstance()->m_pconfig->GetInt(section_name, "version", 0);
    // if (m_version != PROFILE_VERSION)
    // {
    //     printf("bed_mesh: Profile [%s] not compatible with this version\n"
    //            "of bed_mesh.  Profile Version: %d Current Version: %d ",
    //            (section_name.c_str(), m_version, PROFILE_VERSION));
    //     m_incompatible_profiles.push_back(section_name);
    //     return;
    // }
    // std::vector<string> origin_str = Printer::GetInstance()->m_pconfig->get_prefix_options(section_name, "points");
    // strip(origin_str[0]);
    // std::istringstream iss(origin_str[0]);
    // std::string token;
    // while (getline(iss, token, ','))
    // {
    //     m_profiles.points.push_back(atof(token.c_str()));
    // }
    // for (int i = 0; i < m_profiles.points.size(); i++)
    // {
    //     printf("read_points %f\n", m_profiles.points[i]);
    // }
    m_cmd_BED_MESH_PROFILE_help = "Bed Mesh Persistent Storage management";
    Printer::GetInstance()->m_gcode->register_command("BED_MESH_PROFILE", std::bind(&ProfileManager::cmd_BED_MESH_PROFILE, this, std::placeholders::_1), false, m_cmd_BED_MESH_PROFILE_help);
}

ProfileManager::~ProfileManager()
{
}

void ProfileManager::initialize()
{
    _check_incompatible_profiles();
}

std::string ProfileManager::get_current_profile()
{
    return m_current_profile;
}
void ProfileManager::_check_incompatible_profiles()
{
    if (m_incompatible_profiles.size() > 0)
    {
        // configfile = m_printer.lookup_object('configfile')
        // for profile in m_incompatible_profiles:
        //     configfile.remove_section('bed_mesh ' + profile)
        // m_gcode.respond_info(
        //     "The following incompatible profiles have been detected\n"
        //     "and are scheduled for removal:\n%s\n"
        //     "The SAVE_CONFIG command will update the printer config\n"
        //     "file and restart the printer" %
        //     (('\n').join(m_incompatible_profiles)))  // ---??---bed_mesh
    }
}

void ProfileManager::save_profile(std::string prof_name)
{
    ZMesh *z_mesh = m_bedmesh->get_mesh();
    if (!z_mesh)
    {
        printf("Unable to save to profile [ %s ], the bed has not been probed\n", prof_name.c_str());
        return;
    }
    std::vector<std::vector<double>> probed_matrix = z_mesh->get_probed_matrix();
    mesh_config mesh_params = z_mesh->get_mesh_params();
    std::string cfg_name = PROFILE_PREFIX_NAME;
    cfg_name += "_";
    cfg_name += m_bedmesh->m_platform_material;
    cfg_name += "_";
    if (m_bedmesh->m_current_mesh_index)
    {
        cfg_name += to_string(m_bedmesh->m_current_mesh_index);
    }
    else
    {
        cfg_name += prof_name;
    }
    std::string values;
    for (const auto &line : probed_matrix)
    {
        for (const auto &p : line)
        {
            values += std::to_string(p) + ", ";
        }
    }
    values = values.substr(0, values.size() - 2); // 去掉最后一个数据的空格和逗号
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "version", PROFILE_VERSION);
    Printer::GetInstance()->m_pconfig->SetValue(cfg_name, "points", values);
    // 写mesh_params
    values.clear();
    for (auto it = mesh_params.mesh_max.begin(); it != mesh_params.mesh_max.end(); ++it)
    {
        values += std::to_string(*it);
        if (it != mesh_params.mesh_max.end() - 1)
        {
            values += ", ";
        }
    }
    Printer::GetInstance()->m_pconfig->SetValue(cfg_name, "mesh_max", values);
    values.clear();
    for (auto it = mesh_params.mesh_min.begin(); it != mesh_params.mesh_min.end(); ++it)
    {
        values += std::to_string(*it);
        if (it != mesh_params.mesh_min.end() - 1)
        {
            values += ", ";
        }
    }
    Printer::GetInstance()->m_pconfig->SetValue(cfg_name, "mesh_min", values);
    values.clear();
    for (auto it = mesh_params.origin.begin(); it != mesh_params.origin.end(); ++it)
    {
        values += std::to_string(*it);
        if (it != mesh_params.origin.end() - 1)
        {
            values += ", ";
        }
    }
    Printer::GetInstance()->m_pconfig->SetValue(cfg_name, "algo", mesh_params.algo);
    Printer::GetInstance()->m_pconfig->SetDouble(cfg_name, "max_x", mesh_params.max_x);
    Printer::GetInstance()->m_pconfig->SetDouble(cfg_name, "min_x", mesh_params.min_x);
    Printer::GetInstance()->m_pconfig->SetDouble(cfg_name, "max_y", mesh_params.max_y);
    Printer::GetInstance()->m_pconfig->SetDouble(cfg_name, "min_y", mesh_params.min_y);
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "mesh_x_pps", mesh_params.mesh_x_pps);
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "mesh_y_pps", mesh_params.mesh_y_pps);
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "x_count", mesh_params.x_count);
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "y_count", mesh_params.y_count);
    m_current_profile = prof_name;
    // m_bedmesh.update_status();
    Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
    Printer::GetInstance()->m_pconfig->WriteI_specified_Ini(USER_CONFIG_PATH, cfg_name, {});
    printf("Bed Mesh state has been saved to profile [ %s ]\n"
           "for the current session. The SAVE_CONFIG command will\n"
           "update the printer config file and restart the printer.\n",
           prof_name.c_str());
}

bool ProfileManager::save_named_profile(std::string prof_name)
{
    std::string normalized_name = normalize_mesh_profile_name(prof_name);
    if (normalized_name == "")
    {
        return false;
    }
    ZMesh *z_mesh = m_bedmesh->get_mesh();
    if (!z_mesh)
    {
        return false;
    }
    std::vector<std::vector<double>> probed_matrix = z_mesh->get_probed_matrix();
    mesh_config mesh_params = z_mesh->get_mesh_params();
    std::string cfg_name = build_named_profile_section(normalized_name);

    std::string values;
    for (const auto &line : probed_matrix)
    {
        for (const auto &p : line)
        {
            values += std::to_string(p) + ", ";
        }
    }
    if (values.size() >= 2)
    {
        values = values.substr(0, values.size() - 2);
    }
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "version", PROFILE_VERSION);
    Printer::GetInstance()->m_pconfig->SetValue(cfg_name, "points", values);

    values.clear();
    for (auto it = mesh_params.mesh_max.begin(); it != mesh_params.mesh_max.end(); ++it)
    {
        values += std::to_string(*it);
        if (it != mesh_params.mesh_max.end() - 1)
        {
            values += ", ";
        }
    }
    Printer::GetInstance()->m_pconfig->SetValue(cfg_name, "mesh_max", values);
    values.clear();
    for (auto it = mesh_params.mesh_min.begin(); it != mesh_params.mesh_min.end(); ++it)
    {
        values += std::to_string(*it);
        if (it != mesh_params.mesh_min.end() - 1)
        {
            values += ", ";
        }
    }
    Printer::GetInstance()->m_pconfig->SetValue(cfg_name, "mesh_min", values);
    Printer::GetInstance()->m_pconfig->SetValue(cfg_name, "algo", mesh_params.algo);
    Printer::GetInstance()->m_pconfig->SetDouble(cfg_name, "max_x", mesh_params.max_x);
    Printer::GetInstance()->m_pconfig->SetDouble(cfg_name, "min_x", mesh_params.min_x);
    Printer::GetInstance()->m_pconfig->SetDouble(cfg_name, "max_y", mesh_params.max_y);
    Printer::GetInstance()->m_pconfig->SetDouble(cfg_name, "min_y", mesh_params.min_y);
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "mesh_x_pps", mesh_params.mesh_x_pps);
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "mesh_y_pps", mesh_params.mesh_y_pps);
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "x_count", mesh_params.x_count);
    Printer::GetInstance()->m_pconfig->SetInt(cfg_name, "y_count", mesh_params.y_count);
    m_current_profile = normalized_name;
    Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
    Printer::GetInstance()->m_pconfig->WriteI_specified_Ini(USER_CONFIG_PATH, cfg_name, {});
    return true;
}

bool ProfileManager::load_named_profile(std::string prof_name)
{
    std::string normalized_name = normalize_mesh_profile_name(prof_name);
    if (normalized_name == "")
    {
        return false;
    }
    std::string cfg_name = build_named_profile_section(normalized_name);
    if (!Printer::GetInstance()->m_pconfig->IsExistSection(cfg_name))
    {
        return false;
    }

    m_bedmesh->m_bmc->m_mesh_config.mesh_max = parse_pair(cfg_name, "mesh_max", "");
    m_bedmesh->m_bmc->m_mesh_config.mesh_min = parse_pair(cfg_name, "mesh_min", "");
    std::vector<double> points_read = parse_pair(cfg_name, "points", "");
    m_bedmesh->m_bmc->m_mesh_config.algo = Printer::GetInstance()->m_pconfig->GetString(cfg_name, "algo", "");
    m_bedmesh->m_bmc->m_mesh_config.max_x = Printer::GetInstance()->m_pconfig->GetDouble(cfg_name, "max_x", 0.0f);
    m_bedmesh->m_bmc->m_mesh_config.min_x = Printer::GetInstance()->m_pconfig->GetDouble(cfg_name, "min_x", 0.0f);
    m_bedmesh->m_bmc->m_mesh_config.max_y = Printer::GetInstance()->m_pconfig->GetDouble(cfg_name, "max_y", 0.0f);
    m_bedmesh->m_bmc->m_mesh_config.min_y = Printer::GetInstance()->m_pconfig->GetDouble(cfg_name, "min_y", 0.0f);
    m_bedmesh->m_bmc->m_mesh_config.mesh_x_pps = Printer::GetInstance()->m_pconfig->GetInt(cfg_name, "mesh_x_pps", 2);
    m_bedmesh->m_bmc->m_mesh_config.mesh_y_pps = Printer::GetInstance()->m_pconfig->GetInt(cfg_name, "mesh_y_pps", 2);
    m_bedmesh->m_bmc->m_mesh_config.x_count = Printer::GetInstance()->m_pconfig->GetInt(cfg_name, "x_count", 0);
    m_bedmesh->m_bmc->m_mesh_config.y_count = Printer::GetInstance()->m_pconfig->GetInt(cfg_name, "y_count", 0);

    if (m_bedmesh->m_bmc->m_mesh_config.x_count <= 0 || m_bedmesh->m_bmc->m_mesh_config.y_count <= 0)
    {
        return false;
    }

    int expected_points = m_bedmesh->m_bmc->m_mesh_config.x_count * m_bedmesh->m_bmc->m_mesh_config.y_count;
    if ((int)points_read.size() < expected_points)
    {
        return false;
    }
    std::vector<std::vector<double>> matrix(m_bedmesh->m_bmc->m_mesh_config.x_count, std::vector<double>(m_bedmesh->m_bmc->m_mesh_config.y_count));
    for (int i = 0; i < m_bedmesh->m_bmc->m_mesh_config.x_count; i++)
    {
        for (int j = 0; j < m_bedmesh->m_bmc->m_mesh_config.y_count; j++)
        {
            matrix[i][j] = points_read[m_bedmesh->m_bmc->m_mesh_config.y_count * i + j];
        }
    }
    if (m_z_mesh != nullptr)
    {
        delete m_z_mesh;
    }
    m_z_mesh = new ZMesh(m_bedmesh->m_bmc->m_mesh_config);
    m_z_mesh->build_mesh(matrix);
    m_current_profile = normalized_name;
    m_bedmesh->set_mesh(m_z_mesh);
    return true;
}

bool ProfileManager::load_profile_by_name(std::string prof_name)
{
    std::string normalized_name = normalize_mesh_profile_name(prof_name);
    if (normalized_name == "")
    {
        return false;
    }
    if (normalized_name == PROFILE_PREFIX_SUFFIX_NAME)
    {
        return load_profile(PROFILE_PREFIX_SUFFIX_NAME);
    }
    if (load_named_profile(normalized_name))
    {
        return true;
    }
    return load_profile(normalized_name);
}

bool ProfileManager::remove_named_profile(std::string prof_name)
{
    std::string normalized_name = normalize_mesh_profile_name(prof_name);
    if (normalized_name == "")
    {
        return false;
    }
    std::string cfg_name = build_named_profile_section(normalized_name);
    if (!Printer::GetInstance()->m_pconfig->IsExistSection(cfg_name))
    {
        return false;
    }
    Printer::GetInstance()->m_pconfig->DeleteSection(cfg_name);
    Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH);
    return true;
}

bool ProfileManager::load_profile(std::string prof_name)
{
    std::string cfg_name = PROFILE_PREFIX_NAME;
    cfg_name += "_";
    cfg_name += m_bedmesh->m_platform_material;
    cfg_name += "_";
    if (m_bedmesh->m_current_mesh_index)
    {
        cfg_name += to_string(m_bedmesh->m_current_mesh_index);
    }
    else
    {
        cfg_name += prof_name;
    }
    LOG_I("bed mesh cfg_name:%s\n", cfg_name.c_str());
    if (Printer::GetInstance()->m_pconfig->IsExistSection(cfg_name) == false)
    {
        LOG_D("没有参数,不进行读取\n");
        m_bedmesh->set_mesh(nullptr);
        return false;
    }
    // struct mesh_config params;
    m_bedmesh->m_bmc->m_mesh_config.mesh_max = parse_pair(cfg_name, "mesh_max", "");
    m_bedmesh->m_bmc->m_mesh_config.mesh_min = parse_pair(cfg_name, "mesh_min", "");
    std::vector<double> points_read = parse_pair(cfg_name, "points", "");
    std::vector<double> offset_points = parse_pair("bed_mesh", "offset_points", "");
    m_bedmesh->m_bmc->m_mesh_config.algo = Printer::GetInstance()->m_pconfig->GetString(cfg_name, "algo", "");
    m_bedmesh->m_bmc->m_mesh_config.max_x = Printer::GetInstance()->m_pconfig->GetDouble(cfg_name, "max_x", 0.0f);
    m_bedmesh->m_bmc->m_mesh_config.min_x = Printer::GetInstance()->m_pconfig->GetDouble(cfg_name, "min_x", 0.0f);
    m_bedmesh->m_bmc->m_mesh_config.max_y = Printer::GetInstance()->m_pconfig->GetDouble(cfg_name, "max_y", 0.0f);
    m_bedmesh->m_bmc->m_mesh_config.min_y = Printer::GetInstance()->m_pconfig->GetDouble(cfg_name, "min_y", 0.0f);
    m_bedmesh->m_bmc->m_mesh_config.mesh_x_pps = Printer::GetInstance()->m_pconfig->GetInt(cfg_name, "mesh_x_pps", 2);
    m_bedmesh->m_bmc->m_mesh_config.mesh_y_pps = Printer::GetInstance()->m_pconfig->GetInt(cfg_name, "mesh_x_pps", 2);

    m_bedmesh->m_bmc->m_mesh_config.x_count = Printer::GetInstance()->m_pconfig->GetInt(cfg_name, "x_count", 0);
    m_bedmesh->m_bmc->m_mesh_config.y_count = Printer::GetInstance()->m_pconfig->GetInt(cfg_name, "y_count", 0);
    std::vector<std::vector<double>> matrix(m_bedmesh->m_bmc->m_mesh_config.x_count, std::vector<double>(m_bedmesh->m_bmc->m_mesh_config.y_count));
    // if (points_read.size() != offset_points.size())
    // {
    //     LOG_E("offset_points size error! size = %d\n", offset_points.size());
    for (int i = 0; i < m_bedmesh->m_bmc->m_mesh_config.x_count; i++)
    {
        for (int j = 0; j < m_bedmesh->m_bmc->m_mesh_config.y_count; j++)
        {
            matrix[i][j] = points_read[m_bedmesh->m_bmc->m_mesh_config.y_count * i + j];
        }
    }
    // }
    // else
    // {
    //     for (int i = 0; i < m_bedmesh->m_bmc->m_mesh_config.x_count; i++)
    //     {
    //         for (int j = 0; j < m_bedmesh->m_bmc->m_mesh_config.y_count; j++)
    //         {
    //             matrix[i][j] = points_read[m_bedmesh->m_bmc->m_mesh_config.x_count * i + j] + offset_points[m_bedmesh->m_bmc->m_mesh_config.x_count * i + j];
    //         }
    //     }
    // }
    if (m_z_mesh != nullptr)
    {
        delete m_z_mesh;
    }
    m_z_mesh = new ZMesh(m_bedmesh->m_bmc->m_mesh_config); // 注意
    m_z_mesh->build_mesh(matrix);                          // matrix 没有插值的原矩阵
    m_current_profile = prof_name;
    m_bedmesh->set_mesh(m_z_mesh);
    return true;
}

void ProfileManager::remove_profile(std::string prof_name)
{
    // if prof_name in m_profiles:
    //     configfile = m_printer.lookup_object('configfile')
    //     configfile.remove_section('bed_mesh ' + prof_name)
    //     del m_profiles[prof_name]
    //     m_gcode.respond_info(
    //         "Profile [%s] removed from storage for this session.\n"
    //         "The SAVE_CONFIG command will update the printer\n"
    //         "configuration and restart the printer" % (prof_name))
    // else:
    //     m_gcode.respond_info(
    //         "No profile named [%s] to remove" % (prof_name))
}

void ProfileManager::cmd_BED_MESH_PROFILE(GCodeCommand &gcmd)
{
    std::string load_name = normalize_mesh_profile_name(gcmd.get_string("LOAD", ""));
    std::string save_name = normalize_mesh_profile_name(gcmd.get_string("SAVE", ""));
    std::string remove_name = normalize_mesh_profile_name(gcmd.get_string("REMOVE", ""));

    if (load_name != "")
    {
        if (!load_profile_by_name(load_name))
        {
            gcmd.m_respond_info("No mesh profile named [" + load_name + "]", true);
            return;
        }
        gcmd.m_respond_info("Loaded mesh profile [" + load_name + "]", true);
        return;
    }
    if (save_name != "")
    {
        if (save_name == PROFILE_PREFIX_SUFFIX_NAME)
        {
            gcmd.m_respond_info("Profile 'default' is reserved as the fallback profile", true);
            return;
        }
        if (!save_named_profile(save_name))
        {
            gcmd.m_respond_info("Unable to save mesh profile [" + save_name + "]", true);
            return;
        }
        m_bedmesh->set_active_mesh_profile(save_name);
        gcmd.m_respond_info("Saved mesh profile [" + save_name + "]", true);
        return;
    }
    if (remove_name != "")
    {
        if (!remove_named_profile(remove_name))
        {
            gcmd.m_respond_info("No mesh profile named [" + remove_name + "]", true);
            return;
        }
        gcmd.m_respond_info("Removed mesh profile [" + remove_name + "]", true);
        return;
    }
    gcmd.m_respond_info("Invalid BED_MESH_PROFILE syntax. Use LOAD=, SAVE=, or REMOVE=", true);
}
