#ifndef BED_MESH_H
#define BED_MESH_H

#include <string>
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <vector>

#include "probe.h"
#include <functional>

class BedMeshCalibrate;
class ZMesh;
class MoveSplitter;
class ProfileManager;
struct ProfileData
{
    std::vector<double> points;
};
struct orig_config
{
    double radius;
    std::vector<double> origin;
    int rri;
    int x_count;
    int y_count;
    std::vector<double> mesh_min;
    std::vector<double> mesh_max;
    int mesh_x_pps;
    int mesh_y_pps;
    std::string algo;
    double tension;
};

struct mesh_config
{
    double radius;
    std::vector<double> origin;
    int rri;
    std::vector<double> mesh_min;
    std::vector<double> mesh_max;
    int x_count;
    int y_count;
    int mesh_x_pps;
    int mesh_y_pps;
    std::string algo;
    double tension;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
};
class ZMesh
{
private:
public:
    ZMesh(struct mesh_config params);
    ~ZMesh();
    std::map<std::string, std::function<void(std::vector<std::vector<double>>)>> m_interpolation_algos;
    std::function<void(std::vector<std::vector<double>>)> m_sample;
    std::vector<std::vector<double>> m_probed_matrix;//探测测出来的矩阵
    std::vector<std::vector<double>> m_mesh_matrix;//插值后的矩阵,打印补偿实际使用。
    struct mesh_config m_mesh_params;
    double m_avg_z;
    std::vector<double> m_mesh_offsets;
    double m_mesh_x_min;//网格中X最小值
    double m_mesh_x_max;
    double m_mesh_y_min;
    double m_mesh_y_max;
    // Number of points to interpolate per segment
    int m_mesh_x_count;
    int m_mesh_y_count;
    int m_x_mult;
    int m_y_mult;
    double m_mesh_x_dist;//x方向每个点的距离
    double m_mesh_y_dist;

public:
    std::vector<std::vector<double>> get_mesh_matrix();
    std::vector<std::vector<double>> get_probed_matrix();
    struct mesh_config get_mesh_params();
    void print_probed_matrix(std::function<void(std::string, bool)> print_func);
    void print_mesh(std::function<void(std::string, bool)> print_func, double move_z = DBL_MIN);
    void build_mesh(std::vector<std::vector<double>> z_matrix);
    void set_mesh_offsets(std::vector<double> offsets);
    double get_x_coordinate(int index);
    double get_y_coordinate(int index);
    double calc_z(double x, double y);
    std::vector<double> get_z_range();
    std::vector<double> _get_linear_index(double coord, int axis);
    void _sample_direct(std::vector<std::vector<double>> z_matrix);
    void _sample_lagrange(std::vector<std::vector<double>> z_matrix);
    std::vector<std::vector<double>> _get_lagrange_coords();
    double _calc_lagrange(std::vector<double> lpts, double c, int vec, int axis = 0);
    void _sample_bicubic(std::vector<std::vector<double>> z_matrix);
    std::vector<double> _get_x_ctl_pts(double x, double y);
    std::vector<double> _get_y_ctl_pts(double x, double y);
    double _cardinal_spline(std::vector<double> p, double tension);
};
class BedMesh
{
private:
public:
    BedMesh(std::string section_name);
    ~BedMesh();
    void handle_connect();
    void set_mesh(ZMesh *mesh);
    double get_z_factor(double z_pos);
    std::vector<double> get_position();
    bool move(std::vector<double> newpos, double speed);
    bool check_move(std::vector<double> last_pos, std::vector<double>& newpos);
    void get_status(double eventtime);
    ZMesh *get_mesh();
    void update_status();
    void cmd_BED_MESH_SET_INDEX(GCodeCommand &gcmd);
    void cmd_BED_MESH_OUTPUT(GCodeCommand &gcmd);
    void cmd_BED_MESH_MAP(GCodeCommand &gcmd);
    void cmd_BED_MESH_CLEAR(GCodeCommand &gcmd);
    void cmd_BED_MESH_OFFSET(GCodeCommand &gcmd);
    void cmd_BED_MESH_APPLICATIONS(GCodeCommand &gcmd);

public:
    double m_FADE_DISABLE;
    std::vector<double> m_last_position;
    double m_horizontal_move_z;
    double m_fade_start;
    double m_fade_end;
    double m_fade_dist;
    bool m_multi_bed_mesh_enable;
    int m_current_mesh_index;
    std::string m_platform_material;
    bool m_log_fade_complete;
    double m_base_fade_target;
    double m_fade_target;
    std::string cmd_BED_MESH_OUTPUT_help;
    std::string cmd_BED_MESH_MAP_help;
    std::string cmd_BED_MESH_CLEAR_help;
    std::string cmd_BED_MESH_OFFSET_help;
    BedMeshCalibrate *m_bmc = nullptr;
    ZMesh *m_z_mesh = nullptr;
    ZMesh *m_old_z_mesh = nullptr;
    MoveSplitter *m_splitter = nullptr;
    ProfileManager *m_pmgr = nullptr;
    ProbeEndstopWrapperBase *bed_mesh_probe = nullptr;
};

class BedMeshCalibrate
{
private:
public:
    BedMeshCalibrate(std::string section_name, BedMesh *bedmesh);
    ~BedMeshCalibrate();

    void set_mesh_config();
    void _generate_points();
    void print_generated_points();
    void _init_mesh_config(std::string section_name);
    void _verify_algorithm();
    void update_config(GCodeCommand &gcmd);
    std::vector<std::vector<double>> _get_adjusted_points();
    void cmd_BED_MESH_CALIBRATE(GCodeCommand &gcmd);
    std::string probe_finalize(std::vector<double> offsets, std::vector<std::vector<double>> positions);
    void _dump_points(std::vector<std::vector<double>> probed_pts, std::vector<std::vector<double>> corrected_pts, std::vector<double> offsets);

public:
    std::vector<std::string> m_ALGOS;
    struct orig_config m_orig_config;
    double m_radius;
    std::vector<double> m_origin;
    std::vector<double> m_mesh_min;
    std::vector<double> m_mesh_max;
    int m_relative_reference_index;
    std::vector<std::vector<std::vector<double>>> m_faulty_regions;
    std::map<int, std::vector<std::vector<double>>> m_substituted_indices;
    BedMesh *m_bedmesh;
    std::vector<std::vector<double>> m_orig_points;
    std::vector<std::vector<double>> m_points;
    ProbePointsHelper *m_probe_helper;
    std::string cmd_BED_MESH_CALIBRATE_help;
    struct mesh_config m_mesh_config={0};
    ZMesh *m_z_mesh=nullptr;
    std::string m_section_name;
};

class MoveSplitter
{
private:
public:
    MoveSplitter(std::string section_name);
    ~MoveSplitter();

    double m_split_delta_z;
    double m_move_check_distance;
    double m_fade_offset;
    ZMesh *m_z_mesh = nullptr;
    std::vector<double> m_prev_pos;
    std::vector<double> m_next_pos;
    std::vector<double> m_current_pos;
    double m_z_factor;
    double m_z_offset;
    bool m_traverse_complete;
    double m_distance_checked;
    double m_total_move_length;
    bool m_axis_move[8];

public:
    void initialize(ZMesh *mesh, double fade_offset);
    void build_move(std::vector<double> prev_pos, std::vector<double> next_pos, double factor);
    double _calc_z_offset(std::vector<double> pos);
    void _set_next_move(double distance_from_prev);
    std::vector<double> split();
};

class ProfileManager
{
private:
public:
    ProfileManager(std::string section_name, BedMesh *bedmesh);
    ~ProfileManager();

    BedMesh *m_bedmesh;
    std::string m_current_profile;
    std::vector<std::string> m_incompatible_profiles;
    std::string m_cmd_BED_MESH_PROFILE_help;
    std::vector<std::string> m_stored_profs;
    std::string m_name;
    int m_version;
    ZMesh *m_z_mesh;
    ProfileData m_profiles;

public:
    void initialize();
    std::string get_current_profile();
    void save_profile(std::string prof_name);
    void load_profile(std::string prof_name);
    void remove_profile(std::string prof_name);
    void cmd_BED_MESH_PROFILE(GCodeCommand &gcmd);
    void _check_incompatible_profiles();
};
#endif