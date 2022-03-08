#include <LTSMC.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

py::list py_smc_set_connect_timeout(DWORD timems) {
    py::list ret;
    ret.append(smc_set_connect_timeout(timems));
    return ret;
}

py::list py_smc_get_connect_status(WORD ConnectNo) {
    py::list ret;
    ret.append(smc_get_connect_status(ConnectNo));
    return ret;
}

py::list py_smc_set_send_recv_timeout(DWORD SendTimems, DWORD RecvTimems) {
    py::list ret;
    ret.append(smc_set_send_recv_timeout(SendTimems, RecvTimems));
    return ret;
}

py::list py_smc_board_init(WORD ConnectNo, WORD type, char* pconnectstring, DWORD dwBaudRate) {
    py::list ret;
    ret.append(smc_board_init(ConnectNo, type, pconnectstring, dwBaudRate));
    return ret;
}

py::list py_smc_board_init_ex(WORD ConnectNo, WORD type, char* pconnectstring, DWORD dwBaudRate, DWORD dwByteSize, DWORD dwParity, DWORD dwStopBits) {
    py::list ret;
    ret.append(smc_board_init_ex(ConnectNo, type, pconnectstring, dwBaudRate, dwByteSize, dwParity, dwStopBits));
    return ret;
}

py::list py_smc_board_close(WORD ConnectNo) {
    py::list ret;
    ret.append(smc_board_close(ConnectNo));
    return ret;
}

py::list py_smc_soft_reset(WORD ConnectNo) {
    py::list ret;
    ret.append(smc_soft_reset(ConnectNo));
    return ret;
}

py::list py_smc_board_reset(WORD ConnectNo) {
    py::list ret;
    ret.append(smc_board_reset(ConnectNo));
    return ret;
}

py::list py_smc_set_debug_mode(WORD mode, const char* FileName) {
    py::list ret;
    ret.append(smc_set_debug_mode(mode, FileName));
    return ret;
}

py::list py_smc_get_debug_mode() {
    WORD mode = 0;
    char* FileName = new char[1];
    py::list ret;
    ret.append(smc_get_debug_mode(&mode, FileName));
    ret.append(mode);
    ret.append(FileName);
    free(FileName);
    return ret;
}

py::list py_smc_set_connect_debug_time(WORD ConnectNo, DWORD time_s) {
    py::list ret;
    ret.append(smc_set_connect_debug_time(ConnectNo, time_s));
    return ret;
}

py::list py_smc_get_card_version(WORD ConnectNo) {
    DWORD CardVersion = 0;
    py::list ret;
    ret.append(smc_get_card_version(ConnectNo, &CardVersion));
    ret.append(CardVersion);
    return ret;
}

py::list py_smc_get_card_soft_version(WORD ConnectNo) {
    DWORD FirmID = 0;
    DWORD SubFirmID = 0;
    py::list ret;
    ret.append(smc_get_card_soft_version(ConnectNo, &FirmID, &SubFirmID));
    ret.append(FirmID);
    ret.append(SubFirmID);
    return ret;
}

py::list py_smc_get_card_lib_version() {
    DWORD LibVer = 0;
    py::list ret;
    ret.append(smc_get_card_lib_version(&LibVer));
    ret.append(LibVer);
    return ret;
}

py::list py_smc_get_release_version(WORD ConnectNo) {
    char* ReleaseVersion = new char[1];
    py::list ret;
    ret.append(smc_get_release_version(ConnectNo, ReleaseVersion));
    ret.append(ReleaseVersion);
    free(ReleaseVersion);
    return ret;
}

py::list py_smc_get_total_axes(WORD ConnectNo) {
    DWORD TotalAxis = 0;
    py::list ret;
    ret.append(smc_get_total_axes(ConnectNo, &TotalAxis));
    ret.append(TotalAxis);
    return ret;
}

py::list py_smc_get_total_ionum(WORD ConnectNo) {
    WORD TotalIn = 0;
    WORD TotalOut = 0;
    py::list ret;
    ret.append(smc_get_total_ionum(ConnectNo, &TotalIn, &TotalOut));
    ret.append(TotalIn);
    ret.append(TotalOut);
    return ret;
}

py::list py_smc_get_total_adcnum(WORD ConnectNo) {
    WORD TotalIn = 0;
    WORD TotalOut = 0;
    py::list ret;
    ret.append(smc_get_total_adcnum(ConnectNo, &TotalIn, &TotalOut));
    ret.append(TotalIn);
    ret.append(TotalOut);
    return ret;
}

py::list py_smc_format_flash(WORD ConnectNo) {
    py::list ret;
    ret.append(smc_format_flash(ConnectNo));
    return ret;
}

py::list py_smc_rtc_get_time(WORD ConnectNo) {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int min = 0;
    int sec = 0;
    py::list ret;
    ret.append(smc_rtc_get_time(ConnectNo, &year, &month, &day, &hour, &min, &sec));
    ret.append(year);
    ret.append(month);
    ret.append(day);
    ret.append(hour);
    ret.append(min);
    ret.append(sec);
    return ret;
}

py::list py_smc_rtc_set_time(WORD ConnectNo, int year, int month, int day, int hour, int min, int sec) {
    py::list ret;
    ret.append(smc_rtc_set_time(ConnectNo, year, month, day, hour, min, sec));
    return ret;
}

py::list py_smc_set_ipaddr(WORD ConnectNo, const char* IpAddr) {
    py::list ret;
    ret.append(smc_set_ipaddr(ConnectNo, IpAddr));
    return ret;
}

py::list py_smc_get_ipaddr(WORD ConnectNo) {
    char* IpAddr = new char[1];
    py::list ret;
    ret.append(smc_get_ipaddr(ConnectNo, IpAddr));
    ret.append(IpAddr);
    free(IpAddr);
    return ret;
}

py::list py_smc_set_com(WORD ConnectNo, WORD com, DWORD dwBaudRate, WORD wByteSize, WORD wParity, WORD wStopBits) {
    py::list ret;
    ret.append(smc_set_com(ConnectNo, com, dwBaudRate, wByteSize, wParity, wStopBits));
    return ret;
}

py::list py_smc_get_com(WORD ConnectNo, WORD com) {
    DWORD dwBaudRate = 0;
    WORD wByteSize = 0;
    WORD wParity = 0;
    WORD wStopBits = 0;
    py::list ret;
    ret.append(smc_get_com(ConnectNo, com, &dwBaudRate, &wByteSize, &wParity, &wStopBits));
    ret.append(dwBaudRate);
    ret.append(wByteSize);
    ret.append(wParity);
    ret.append(wStopBits);
    return ret;
}

py::list py_smc_write_sn(WORD ConnectNo, uint64 sn) {
    py::list ret;
    ret.append(smc_write_sn(ConnectNo, sn));
    return ret;
}

py::list py_smc_read_sn(WORD ConnectNo) {
    uint64 sn = 0;
    py::list ret;
    ret.append(smc_read_sn(ConnectNo, &sn));
    ret.append(sn);
    return ret;
}

py::list py_smc_write_sn_numstring(WORD ConnectNo, const char* sn_str) {
    py::list ret;
    ret.append(smc_write_sn_numstring(ConnectNo, sn_str));
    return ret;
}

py::list py_smc_read_sn_numstring(WORD ConnectNo) {
    char* sn_str = new char[1];
    py::list ret;
    ret.append(smc_read_sn_numstring(ConnectNo, sn_str));
    ret.append(sn_str);
    return ret;
}

py::list py_smc_write_password(WORD ConnectNo, const char* str_pass) {
    py::list ret;
    ret.append(smc_write_password(ConnectNo, str_pass));
    return ret;
}

py::list py_smc_check_password(WORD ConnectNo, const char* str_pass) {
    py::list ret;
    ret.append(smc_check_password(ConnectNo, str_pass));
    return ret;
}

py::list py_smc_enter_password(WORD ConnectNo, const char* str_pass) {
    py::list ret;
    ret.append(smc_enter_password(ConnectNo, str_pass));
    return ret;
}

py::list py_smc_modify_password(WORD ConnectNo, const char* spassold, const char* spass) {
    py::list ret;
    ret.append(smc_modify_password(ConnectNo, spassold, spass));
    return ret;
}

py::list py_smc_download_parafile(WORD ConnectNo, const char* FileName) {
    py::list ret;
    ret.append(smc_download_parafile(ConnectNo, FileName));
    return ret;
}

py::list py_smc_upload_parafile(WORD ConnectNo, const char* FileName) {
    py::list ret;
    ret.append(smc_upload_parafile(ConnectNo, FileName));
    return ret;
}

py::list py_smc_set_el_mode(WORD ConnectNo, WORD axis, WORD enable, WORD el_logic, WORD el_mode) {
    py::list ret;
    ret.append(smc_set_el_mode(ConnectNo, axis, enable, el_logic, el_mode));
    return ret;
}

py::list py_smc_get_el_mode(WORD ConnectNo, WORD axis) {
    WORD enable = 0;
    WORD el_logic = 0;
    WORD el_mode = 0;
    py::list ret;
    ret.append(smc_get_el_mode(ConnectNo, axis, &enable, &el_logic, &el_mode));
    ret.append(enable);
    ret.append(el_logic);
    ret.append(el_mode);
    return ret;
}

py::list py_smc_set_emg_mode(WORD ConnectNo, WORD axis, WORD enable, WORD emg_logic) {
    py::list ret;
    ret.append(smc_set_emg_mode(ConnectNo, axis, enable, emg_logic));
    return ret;
}

py::list py_smc_get_emg_mode(WORD ConnectNo, WORD axis) {
    WORD enable = 0;
    WORD emg_logic = 0;
    py::list ret;
    ret.append(smc_get_emg_mode(ConnectNo, axis, &enable, &emg_logic));
    ret.append(enable);
    ret.append(emg_logic);
    return ret;
}

py::list py_smc_set_softlimit_unit(WORD ConnectNo, WORD axis, WORD enable, WORD source_sel, WORD SL_action, double N_limit, double P_limit) {
    py::list ret;
    ret.append(smc_set_softlimit_unit(ConnectNo, axis, enable, source_sel, SL_action, N_limit, P_limit));
    return ret;
}

py::list py_smc_get_softlimit_unit(WORD ConnectNo, WORD axis) {
    WORD enable = 0;
    WORD source_sel = 0;
    WORD SL_action = 0;
    double N_limit = 0;
    double P_limit = 0;
    py::list ret;
    ret.append(smc_get_softlimit_unit(ConnectNo, axis, &enable, &source_sel, &SL_action, &N_limit, &P_limit));
    ret.append(enable);
    ret.append(source_sel);
    ret.append(SL_action);
    ret.append(N_limit);
    ret.append(P_limit);
    return ret;
}

py::list py_smc_set_pulse_outmode(WORD ConnectNo, WORD axis, WORD outmode) {
    py::list ret;
    ret.append(smc_set_pulse_outmode(ConnectNo, axis, outmode));
    return ret;
}

py::list py_smc_get_pulse_outmode(WORD ConnectNo, WORD axis) {
    WORD outmode = 0;
    py::list ret;
    ret.append(smc_get_pulse_outmode(ConnectNo, axis, &outmode));
    ret.append(outmode);
    return ret;
}

py::list py_smc_set_equiv(WORD ConnectNo, WORD axis, double equiv) {
    py::list ret;
    ret.append(smc_set_equiv(ConnectNo, axis, equiv));
    return ret;
}

py::list py_smc_get_equiv(WORD ConnectNo, WORD axis) {
    double equiv = 0.0;
    py::list ret;
    ret.append(smc_get_equiv(ConnectNo, axis, &equiv));
    ret.append(equiv);
    return ret;
}

py::list py_smc_set_backlash_unit(WORD ConnectNo, WORD axis, double backlash) {
    py::list ret;
    ret.append(smc_set_backlash_unit(ConnectNo, axis, backlash));
    return ret;
}

py::list py_smc_get_backlash_unit(WORD ConnectNo, WORD axis) {
    double backlash = 0.0;
    py::list ret;
    ret.append(smc_get_backlash_unit(ConnectNo, axis, &backlash));
    ret.append(backlash);
    return ret;
}

py::list py_smc_set_axis_io_map(WORD ConnectNo, WORD Axis, WORD IoType, WORD MapIoType, WORD MapIoIndex, double Filter) {
    py::list ret;
    ret.append(smc_set_axis_io_map(ConnectNo, Axis, IoType, MapIoType, MapIoIndex, Filter));
    return ret;
}

py::list py_smc_get_axis_io_map(WORD ConnectNo, WORD Axis, WORD IoType) {
    WORD MapIoType = 0;
    WORD MapIoIndex = 0;
    double Filter = 0.0;
    py::list ret;
    ret.append(smc_get_axis_io_map(ConnectNo, Axis, IoType, &MapIoType, &MapIoIndex, &Filter));
    ret.append(MapIoType);
    ret.append(MapIoIndex);
    ret.append(Filter);
    return ret;
}

py::list py_smc_set_profile_unit(WORD ConnectNo, WORD axis, double Min_Vel, double Max_Vel, double Tacc, double Tdec, double Stop_Vel) {
    py::list ret;
    ret.append(smc_set_profile_unit(ConnectNo, axis, Min_Vel, Max_Vel, Tacc, Tdec, Stop_Vel));
    return ret;
}

py::list py_smc_get_profile_unit(WORD ConnectNo, WORD axis) {
    double Min_Vel = 0.0;
    double Max_Vel = 0.0;
    double Tacc = 0.0;
    double Tdec = 0.0;
    double Stop_Vel = 0.0;
    py::list ret;
    ret.append(smc_get_profile_unit(ConnectNo, axis, &Min_Vel, &Max_Vel, &Tacc, &Tdec, &Stop_Vel));
    ret.append(Min_Vel);
    ret.append(Max_Vel);
    ret.append(Tacc);
    ret.append(Tdec);
    ret.append(Stop_Vel);
    return ret;
}

py::list py_smc_set_profile_unit_acc(WORD ConnectNo, WORD axis, double Min_Vel, double Max_Vel, double acc, double dec, double Stop_Vel) {
    py::list ret;
    ret.append(smc_set_profile_unit_acc(ConnectNo, axis, Min_Vel, Max_Vel, acc, dec, Stop_Vel));
    return ret;
}

py::list py_smc_get_profile_unit_acc(WORD ConnectNo, WORD axis) {
    double Min_Vel = 0.0;
    double Max_Vel = 0.0;
    double acc = 0.0;
    double dec = 0.0;
    double Stop_Vel = 0.0;
    py::list ret;
    ret.append(smc_get_profile_unit_acc(ConnectNo, axis, &Min_Vel, &Max_Vel, &acc, &dec, &Stop_Vel));
    ret.append(Min_Vel);
    ret.append(Max_Vel);
    ret.append(acc);
    ret.append(dec);
    ret.append(Stop_Vel);
    return ret;
}

py::list py_smc_set_s_profile(WORD ConnectNo, WORD axis, WORD s_mode, double s_para) {
    py::list ret;
    ret.append(smc_set_s_profile(ConnectNo, axis, s_mode, s_para));
    return ret;
}

py::list py_smc_get_s_profile(WORD ConnectNo, WORD axis, WORD s_mode) {
    double s_para = 0.0;
    py::list ret;
    ret.append(smc_get_s_profile(ConnectNo, axis, s_mode, &s_para));
    ret.append(s_para);
    return ret;
}

py::list py_smc_set_dec_stop_time(WORD ConnectNo, WORD axis, double time) {
    py::list ret;
    ret.append(smc_set_dec_stop_time(ConnectNo, axis, time));
    return ret;
}

py::list py_smc_get_dec_stop_time(WORD ConnectNo, WORD axis) {
    double time;
    py::list ret;
    ret.append(smc_get_dec_stop_time(ConnectNo, axis, &time));
    ret.append(time);
    return ret;
}

py::list py_smc_pmove_unit(WORD ConnectNo, WORD axis, double Dist, WORD posi_mode) {
    py::list ret;
    ret.append(smc_pmove_unit(ConnectNo, axis, Dist, posi_mode));
    return ret;
}

py::list py_smc_vmove(WORD ConnectNo, WORD axis, WORD dir) {
    py::list ret;
    ret.append(smc_vmove(ConnectNo, axis, dir));
    return ret;
}

py::list py_smc_change_speed_unit(WORD ConnectNo, WORD axis, double New_Vel, double Taccdec) {
    py::list ret;
    ret.append(smc_change_speed_unit(ConnectNo, axis, New_Vel, Taccdec));
    return ret;
}

py::list py_smc_reset_target_position_unit(WORD ConnectNo, WORD axis, double New_Pos) {
    py::list ret;
    ret.append(smc_reset_target_position_unit(ConnectNo, axis, New_Pos));
    return ret;
}

py::list py_smc_update_target_position_unit(WORD ConnectNo, WORD axis, double New_Pos) {
    py::list ret;
    ret.append(smc_update_target_position_unit(ConnectNo, axis, New_Pos));
    return ret;
}

py::list py_smc_pmove_unit_extern(WORD ConnectNo, WORD axis, double MidPos, double TargetPos, double Min_Vel, double Max_Vel, double stop_Vel, double acc, double dec, double smooth_time, WORD posi_mode) {
    py::list ret;
    ret.append(smc_pmove_unit_extern(ConnectNo, axis, MidPos, TargetPos, Min_Vel, Max_Vel, stop_Vel, acc, dec, smooth_time, posi_mode));
    return ret;
}

py::list py_smc_set_plan_mode(WORD ConnectNo, WORD axis, WORD mode) {
    py::list ret;
    ret.append(smc_set_plan_mode(ConnectNo, axis, mode));
    return ret;
}

py::list py_smc_get_plan_mode(WORD ConnectNo, WORD axis) {
    WORD mode = 0;
    py::list ret;
    ret.append(smc_get_plan_mode(ConnectNo, axis, &mode));
    ret.append(mode);
    return ret;
}

py::list py_smc_pmove_sin_unit(WORD ConnectNo, WORD axis, double Dist, WORD posi_mode, double MaxVel, double MaxAcc) {
    py::list ret;
    ret.append(smc_pmove_sin_unit(ConnectNo, axis, Dist, posi_mode, MaxVel, MaxAcc));
    return ret;
}

py::list py_smc_pmove_change_pos_speed_config(WORD ConnectNo, WORD axis, double tar_vel, double tar_rel_pos, WORD trig_mode, WORD source) {
    py::list ret;
    ret.append(smc_pmove_change_pos_speed_config(ConnectNo, axis, tar_vel, tar_rel_pos, trig_mode, source));
    return ret;
}

py::list py_smc_get_pmove_change_pos_speed_config(WORD ConnectNo, WORD axis) {
    double tar_vel = 0.0;
    double tar_rel_pos = 0.0;
    WORD trig_mode = 0;
    WORD source = 0;
    py::list ret;
    ret.append(smc_get_pmove_change_pos_speed_config(ConnectNo, axis, &tar_vel, &tar_rel_pos, &trig_mode, &source));
    ret.append(tar_vel);
    ret.append(tar_rel_pos);
    ret.append(trig_mode);
    ret.append(source);
    return ret;
}

py::list py_smc_pmove_change_pos_speed_enable(WORD ConnectNo, WORD axis, WORD enable) {
    py::list ret;
    ret.append(smc_pmove_change_pos_speed_enable(ConnectNo, axis, enable));
    return ret;
}

py::list py_smc_get_pmove_change_pos_speed_enable(WORD ConnectNo, WORD axis) {
    WORD enable = 0;
    py::list ret;
    ret.append(smc_get_pmove_change_pos_speed_enable(ConnectNo, axis, &enable));
    ret.append(enable);
    return ret;
}

py::list py_smc_get_pmove_change_pos_speed_state(WORD ConnectNo, WORD axis) {
    WORD trig_num = 0;
    double trig_pos = 0.0;
    py::list ret;
    ret.append(smc_get_pmove_change_pos_speed_state(ConnectNo, axis, &trig_num, &trig_pos));
    ret.append(trig_num);
    ret.append(trig_pos);
    return ret;
}

py::list py_smc_set_home_pin_logic(WORD ConnectNo, WORD axis, WORD org_logic, double filter) {
    py::list ret;
    ret.append(smc_set_home_pin_logic(ConnectNo, axis, org_logic, filter));
    return ret;
}

py::list py_smc_get_home_pin_logic(WORD ConnectNo, WORD axis) {
    WORD org_logic = 0;
    double filter = 0.0;
    py::list ret;
    ret.append(smc_get_home_pin_logic(ConnectNo, axis, &org_logic, &filter));
    ret.append(org_logic);
    ret.append(filter);
    return ret;
}

py::list py_smc_set_ez_mode(WORD ConnectNo, WORD axis, WORD ez_logic, WORD ez_mode, double filter) {
    py::list ret;
    ret.append(smc_set_ez_mode(ConnectNo, axis, ez_logic, ez_mode, filter));
    return ret;
}

py::list py_smc_get_ez_mode(WORD ConnectNo, WORD axis) {
    WORD ez_logic = 0;
    WORD ez_mode = 0;
    double filter = 0.0;
    py::list ret;
    ret.append(smc_get_ez_mode(ConnectNo, axis, &ez_logic, &ez_mode, &filter));
    ret.append(ez_logic);
    ret.append(filter);
    return ret;
}

py::list py_smc_set_homemode(WORD ConnectNo, WORD axis, WORD home_dir, double vel_mode, WORD mode, WORD pos_source) {
    py::list ret;
    ret.append(smc_set_homemode(ConnectNo, axis, home_dir, vel_mode, mode, pos_source));
    return ret;
}

py::list py_smc_get_homemode(WORD ConnectNo, WORD axis) {
    WORD home_dir = 0;
    double vel_mode = 0.0;
    WORD home_mode = 0;
    WORD pos_source = 0;
    py::list ret;
    ret.append(smc_get_homemode(ConnectNo, axis, &home_dir, &vel_mode, &home_mode, &pos_source));
    ret.append(home_dir);
    ret.append(vel_mode);
    ret.append(home_mode);
    ret.append(pos_source);
    return ret;
}

py::list py_smc_set_homespeed_unit(WORD ConnectNo, WORD axis, double homespeed) {
    py::list ret;
    ret.append(smc_set_homespeed_unit(ConnectNo, axis, homespeed));
    return ret;
}

py::list py_smc_get_homespeed_unit(WORD ConnectNo, WORD axis) {
    double homespeed = 0.0;
    py::list ret;
    ret.append(smc_get_homespeed_unit(ConnectNo, axis, &homespeed));
    ret.append(homespeed);
    return ret;
}

py::list py_smc_set_home_profile_unit(WORD ConnectNo, WORD axis, double Low_Vel, double High_Vel, double Tacc, double Tdec) {
    py::list ret;
    ret.append(smc_set_home_profile_unit(ConnectNo, axis, Low_Vel, High_Vel, Tacc, Tdec));
    return ret;
}

py::list py_smc_get_home_profile_unit(WORD ConnectNo, WORD axis) {
    double Low_Vel = 0;
    double High_Vel = 0;
    double Tacc = 0;
    double Tdec = 0;
    py::list ret;
    ret.append(smc_get_home_profile_unit(ConnectNo, axis, &Low_Vel, &High_Vel, &Tacc, &Tdec));
    ret.append(Low_Vel);
    ret.append(High_Vel);
    ret.append(Tacc);
    ret.append(Tdec);
    return ret;
}

py::list py_smc_set_el_home(WORD ConnectNo, WORD axis, WORD mode) {
    py::list ret;
    ret.append(smc_set_el_home(ConnectNo, axis, mode));
    return ret;
}

py::list py_smc_set_home_position_unit(WORD ConnectNo, WORD axis, WORD enable, double position) {
    py::list ret;
    ret.append(smc_set_home_position_unit(ConnectNo, axis, enable, position));
    return ret;
}

py::list py_smc_get_home_position_unit(WORD ConnectNo, WORD axis) {
    WORD enable = 0;
    double position = 0.0;
    py::list ret;
    ret.append(smc_get_home_position_unit(ConnectNo, axis, &enable, &position));
    ret.append(enable);
    ret.append(position);
    return ret;
}

py::list py_smc_home_move(WORD ConnectNo, WORD axis) {
    py::list ret;
    ret.append(smc_home_move(ConnectNo, axis));
    return ret;
}

py::list py_smc_get_home_result(WORD ConnectNo, WORD axis) {
    WORD state = 0;
    py::list ret;
    ret.append(smc_get_home_result(ConnectNo, axis, &state));
    ret.append(state);
    return ret;
}

// TO DO
//  py::list py_smc_pvt_table_unit(WORD ConnectNo, WORD iaxis, DWORD count, double* pTime, double* pPos, double* pVel) {
//      py::list ret;
//      ret.append(smc_pvt_table_unit(ConnectNo, iaxis, count, pTime, pPos, pVel));
//      return ret;
//  }

// py::list py_smc_pts_table_unit(WORD ConnectNo, WORD iaxis, DWORD count, double* pTime, double* pPos, double* pPercent) {
//     py::list ret;
//     ret.append(smc_pts_table_unit(ConnectNo, iaxis, count, pTime, pPos, pPercent));
//     return ret;
// }

// py::list py_smc_pvts_table_unit(WORD ConnectNo, WORD iaxis, DWORD count, double* pTime, double* pPos, double velBegin, double velEnd) {
//     py::list ret;
//     ret.append(smc_pvts_table_unit(ConnectNo, iaxis, count, pTime, pPos, velBegin, velEnd));
//     return ret;
// }

// py::list py_smc_ptt_table_unit(WORD ConnectNo, WORD iaxis, DWORD count, double* pTime, double* pPos) {
//     py::list ret;
//     ret.append(smc_ptt_table_unit(ConnectNo, iaxis, count, pTime, pPos));
//     return ret;
// }

// py::list py_smc_pvt_move(WORD ConnectNo, WORD AxisNum, WORD* AxisList) {
//     py::list ret;
//     ret.append(smc_pvt_move(ConnectNo, AxisNum, AxisList));
//     return ret;
// }

// py::list py_smc_cam_table_unit(WORD ConnectNo, WORD MasterAxisNo, WORD SlaveAxisNo, DWORD Count, double* pMasterPos, double* pSlavePos, WORD SrcMode) {
//     py::list ret;
//     ret.append(smc_cam_table_unit(ConnectNo, MasterAxisNo, SlaveAxisNo, Count, pMasterPos, pSlavePos, SrcMode));
//     return ret;
// }

py::list py_smc_cam_move(WORD ConnectNo, WORD AxisNo) {
    py::list ret;
    ret.append(smc_cam_move(ConnectNo, AxisNo));
    return ret;
}

py::list py_smc_sine_oscillate_unit(WORD ConnectNo, WORD Axis, double Amplitude, double Frequency) {
    py::list ret;
    ret.append(smc_sine_oscillate_unit(ConnectNo, Axis, Amplitude, Frequency));
    return ret;
}

py::list py_smc_sine_oscillate_stop(WORD ConnectNo, WORD Axis) {
    py::list ret;
    ret.append(smc_sine_oscillate_stop(ConnectNo, Axis));
    return ret;
}

py::list py_smc_handwheel_set_axislist(WORD ConnectNo, WORD AxisSelIndex, WORD AxisNum, WORD* AxisList) {
    py::list ret;
    ret.append(smc_handwheel_set_axislist(ConnectNo, AxisSelIndex, AxisNum, AxisList));
    return ret;
}

py::list py_smc_handwheel_get_axislist(WORD ConnectNo, WORD AxisSelIndex) {
    WORD AxisNum = 0;
    WORD AxisList = 0;
    py::list ret;
    ret.append(smc_handwheel_get_axislist(ConnectNo, AxisSelIndex, &AxisNum, &AxisList));
    ret.append(AxisNum);
    ret.append(AxisList);
    return ret;
}

py::list py_smc_handwheel_set_ratiolist(WORD ConnectNo, WORD AxisSelIndex, WORD StartRatioIndex, WORD RatioSelNum, double* RatioList) {
    py::list ret;
    ret.append(smc_handwheel_set_ratiolist(ConnectNo, AxisSelIndex, StartRatioIndex, RatioSelNum, RatioList));
    return ret;
}

py::list py_smc_handwheel_get_ratiolist(WORD ConnectNo, WORD AxisSelIndex, WORD StartRatioIndex, WORD RatioSelNum) {
    double RatioList = 0;
    py::list ret;
    ret.append(smc_handwheel_get_ratiolist(ConnectNo, AxisSelIndex, StartRatioIndex, RatioSelNum, &RatioList));
    ret.append(RatioList);
    return ret;
}

py::list py_smc_handwheel_set_mode(WORD ConnectNo, WORD InMode, WORD IfHardEnable) {
    py::list ret;
    ret.append(smc_handwheel_set_mode(ConnectNo, InMode, IfHardEnable));
    return ret;
}

py::list py_smc_handwheel_get_mode(WORD ConnectNo) {
    WORD InMode = 0;
    WORD IfHardEnable = 0;
    py::list ret;
    ret.append(smc_handwheel_get_mode(ConnectNo, &InMode, &IfHardEnable));
    ret.append(InMode);
    ret.append(IfHardEnable);
    return ret;
}

py::list py_smc_handwheel_set_index(WORD ConnectNo, WORD AxisSelIndex, WORD RatioSelIndex) {
    py::list ret;
    ret.append(smc_handwheel_set_index(ConnectNo, AxisSelIndex, RatioSelIndex));
    return ret;
}

py::list py_smc_handwheel_get_index(WORD ConnectNo) {
    WORD AxisSelIndex = 0;
    WORD RatioSelIndex = 0;
    py::list ret;
    ret.append(smc_handwheel_get_index(ConnectNo, &AxisSelIndex, &RatioSelIndex));
    ret.append(AxisSelIndex);
    ret.append(RatioSelIndex);
    return ret;
}

py::list py_smc_handwheel_move(WORD ConnectNo, WORD ForceMove) {
    py::list ret;
    ret.append(smc_handwheel_move(ConnectNo, ForceMove));
    return ret;
}

py::list py_smc_handwheel_stop(WORD ConnectNo) {
    py::list ret;
    ret.append(smc_handwheel_stop(ConnectNo));
    return ret;
}

py::list py_smc_set_handwheel_inmode(WORD ConnectNo, WORD axis, WORD inmode, long multi, double vh) {
    py::list ret;
    ret.append(smc_set_handwheel_inmode(ConnectNo, axis, inmode, multi, vh));
    return ret;
}

py::list py_smc_get_handwheel_inmode(WORD ConnectNo, WORD axis) {
    WORD inmode = 0;
    long multi = 0;
    double vh = 0;
    py::list ret;
    ret.append(smc_get_handwheel_inmode(ConnectNo, axis, &inmode, &multi, &vh));
    ret.append(inmode);
    ret.append(multi);
    ret.append(vh);
    return ret;
}

py::list py_smc_set_handwheel_inmode_extern(WORD ConnectNo, WORD inmode, WORD AxisNum, WORD* AxisList, int* multi) {
    py::list ret;
    ret.append(smc_set_handwheel_inmode_extern(ConnectNo, inmode, AxisNum, AxisList, multi));
    return ret;
}

py::list py_smc_get_handwheel_inmode_extern(WORD ConnectNo) {
    WORD inmode = 0;
    WORD AxisNum = 0;
    WORD AxisList = 0;
    int multi = 0;
    py::list ret;
    ret.append(smc_get_handwheel_inmode_extern(ConnectNo, &inmode, &AxisNum, &AxisList, &multi));
    ret.append(inmode);
    ret.append(AxisNum);
    ret.append(AxisList);
    ret.append(multi);
    return ret;
}

py::list py_smc_set_handwheel_inmode_decimals(WORD ConnectNo, WORD axis, WORD inmode, double multi, double vh) {
    py::list ret;
    ret.append(smc_set_handwheel_inmode_decimals(ConnectNo, axis, inmode, multi, vh));
    return ret;
}

py::list py_smc_get_handwheel_inmode_decimals(WORD ConnectNo, WORD axis) {
    WORD inmode = 0;
    double multi = 0;
    double vh = 0;
    py::list ret;
    ret.append(smc_get_handwheel_inmode_decimals(ConnectNo, axis, &inmode, &multi, &vh));
    ret.append(inmode);
    ret.append(multi);
    ret.append(vh);
    return ret;
}

py::list py_smc_set_handwheel_inmode_extern_decimals(WORD ConnectNo, WORD inmode, WORD AxisNum, WORD* AxisList, double* multi) {
    py::list ret;
    ret.append(smc_set_handwheel_inmode_extern_decimals(ConnectNo, inmode, AxisNum, AxisList, multi));
    return ret;
}

py::list py_smc_get_handwheel_inmode_extern_decimals(WORD ConnectNo) {
    WORD inmode = 0;
    WORD AxisNum = 0;
    WORD AxisList = 0;
    double multi = 0;
    py::list ret;
    ret.append(smc_get_handwheel_inmode_extern_decimals(ConnectNo, &inmode, &AxisNum, &AxisList, &multi));
    ret.append(inmode);
    ret.append(AxisNum);
    ret.append(AxisList);
    ret.append(multi);
    return ret;
}

py::list py_smc_set_vector_profile_unit(WORD ConnectNo, WORD Crd, double Min_Vel, double Max_Vel, double Tacc, double Tdec, double Stop_Vel) {
    py::list ret;
    ret.append(smc_set_vector_profile_unit(ConnectNo, Crd, Min_Vel, Max_Vel, Tacc, Tdec, Stop_Vel));
    return ret;
}

py::list py_smc_get_vector_profile_unit(WORD ConnectNo, WORD Crd) {
    double Min_Vel = 0;
    double Max_Vel = 0;
    double Tacc = 0;
    double Tdec = 0;
    double Stop_Vel = 0;
    py::list ret;
    ret.append(smc_get_vector_profile_unit(ConnectNo, Crd, &Min_Vel, &Max_Vel, &Tacc, &Tdec, &Stop_Vel));
    ret.append(Min_Vel);
    ret.append(Max_Vel);
    ret.append(Tacc);
    ret.append(Tdec);
    ret.append(Stop_Vel);
    return ret;
}

py::list py_smc_set_vector_profile_unit_acc(WORD ConnectNo, WORD Crd, double Min_Vel, double Max_Vel, double acc, double dec, double Stop_Vel) {
    py::list ret;
    ret.append(smc_set_vector_profile_unit_acc(ConnectNo, Crd, Min_Vel, Max_Vel, acc, dec, Stop_Vel));
    return ret;
}

py::list py_smc_get_vector_profile_unit_acc(WORD ConnectNo, WORD Crd) {
    double Min_Vel = 0;
    double Max_Vel = 0;
    double acc = 0;
    double dec = 0;
    double Stop_Vel = 0;
    py::list ret;
    ret.append(smc_get_vector_profile_unit_acc(ConnectNo, Crd, &Min_Vel, &Max_Vel, &acc, &dec, &Stop_Vel));
    ret.append(Min_Vel);
    ret.append(Max_Vel);
    ret.append(acc);
    ret.append(dec);
    ret.append(Stop_Vel);
    return ret;
}

py::list py_smc_set_vector_s_profile(WORD ConnectNo, WORD Crd, WORD s_mode, double s_para) {
    py::list ret;
    ret.append(smc_set_vector_s_profile(ConnectNo, Crd, s_mode, s_para));
    return ret;
}

py::list py_smc_get_vector_s_profile(WORD ConnectNo, WORD Crd, WORD s_mode) {
    double s_para;
    py::list ret;
    ret.append(smc_get_vector_s_profile(ConnectNo, Crd, s_mode, &s_para));
    ret.append(s_para);
    return ret;
}

py::list py_smc_set_vector_dec_stop_time(WORD ConnectNo, WORD Crd, double time) {
    py::list ret;
    ret.append(smc_set_vector_dec_stop_time(ConnectNo, Crd, time));
    return ret;
}

py::list py_smc_get_vector_dec_stop_time(WORD ConnectNo, WORD Crd) {
    double time = 0;
    py::list ret;
    ret.append(smc_get_vector_dec_stop_time(ConnectNo, Crd, &time));
    ret.append(time);
    return ret;
}

py::list py_smc_line_unit(WORD ConnectNo, WORD Crd, WORD AxisNum, WORD* AxisList, double* Dist, WORD posi_mode) {
    py::list ret;
    ret.append(smc_line_unit(ConnectNo, Crd, AxisNum, AxisList, Dist, posi_mode));
    return ret;
}

py::list py_smc_arc_move_center_unit(WORD ConnectNo, WORD Crd, WORD AxisNum, WORD* AxisList, double* Target_Pos, double* Cen_Pos, WORD Arc_Dir, long Circle, WORD posi_mode) {
    py::list ret;
    ret.append(smc_arc_move_center_unit(ConnectNo, Crd, AxisNum, AxisList, Target_Pos, Cen_Pos, Arc_Dir, Circle, posi_mode));
    return ret;
}

py::list py_smc_arc_move_radius_unit(WORD ConnectNo, WORD Crd, WORD AxisNum, WORD* AxisList, double* Target_Pos, double Arc_Radius, WORD Arc_Dir, long Circle, WORD posi_mode) {
    py::list ret;
    ret.append(smc_arc_move_radius_unit(ConnectNo, Crd, AxisNum, AxisList, Target_Pos, Arc_Radius, Arc_Dir, Circle, posi_mode));
    return ret;
}

py::list py_smc_arc_move_3points_unit(WORD ConnectNo, WORD Crd, WORD AxisNum, WORD* AxisList, double* Target_Pos, double* Mid_Pos, long Circle, WORD posi_mode) {
    py::list ret;
    ret.append(smc_arc_move_3points_unit(ConnectNo, Crd, AxisNum, AxisList, Target_Pos, Mid_Pos, Circle, posi_mode));
    return ret;
}

py::list py_smc_conti_set_lookahead_mode(WORD ConnectNo, WORD Crd, WORD enable, long LookaheadSegments, double PathError, double LookaheadAcc) {
    py::list ret;
    ret.append(smc_conti_set_lookahead_mode(ConnectNo, Crd, enable, LookaheadSegments, PathError, LookaheadAcc));
    return ret;
}

py::list py_smc_conti_get_lookahead_mode(WORD ConnectNo, WORD Crd) {
    WORD enable = 0;
    long LookaheadSegments = 0;
    double PathError = 0;
    double LookaheadAcc = 0;
    py::list ret;
    ret.append(smc_conti_get_lookahead_mode(ConnectNo, Crd, &enable, &LookaheadSegments, &PathError, &LookaheadAcc));
    ret.append(enable);
    ret.append(LookaheadSegments);
    ret.append(PathError);
    ret.append(LookaheadAcc);
    return ret;
}

py::list py_smc_set_arc_limit(WORD ConnectNo, WORD Crd, WORD Enable, double MaxCenAcc = 0, double MaxArcError = 0) {
    py::list ret;
    ret.append(smc_set_arc_limit(ConnectNo, Crd, Enable, 0, 0));
    return ret;
}

py::list py_smc_get_arc_limit(WORD ConnectNo, WORD Crd) {
    WORD Enable = 0;
    double MaxCenAcc = 0;
    double MaxArcError = 0;
    py::list ret;
    ret.append(smc_get_arc_limit(ConnectNo, Crd, &Enable, &MaxCenAcc, &MaxCenAcc));
    ret.append(Enable);
    ret.append(MaxCenAcc);
    ret.append(MaxArcError);
    return ret;
}

py::list py_smc_conti_open_list(WORD ConnectNo, WORD Crd, WORD AxisNum, std::vector<WORD> AxisList) {
    py::list ret;
    WORD* AxisList_arr = &AxisList[0];
    ret.append(smc_conti_open_list(ConnectNo, Crd, AxisNum, AxisList_arr));
    return ret;
}

py::list py_smc_conti_close_list(WORD ConnectNo, WORD Crd) {
    py::list ret;
    ret.append(smc_conti_close_list(ConnectNo, Crd));
    return ret;
}

py::list py_smc_conti_stop_list(WORD ConnectNo, WORD Crd, WORD stop_mode) {
    py::list ret;
    ret.append(smc_conti_stop_list(ConnectNo, Crd, stop_mode));
    return ret;
}

py::list py_smc_conti_pause_list(WORD ConnectNo, WORD Crd) {
    py::list ret;
    ret.append(smc_conti_pause_list(ConnectNo, Crd));
    return ret;
}

py::list py_smc_conti_start_list(WORD ConnectNo, WORD Crd) {
    py::list ret;
    ret.append(smc_conti_start_list(ConnectNo, Crd));
    return ret;
}

py::list py_smc_conti_change_speed_ratio(WORD ConnectNo, WORD Crd, double percent) {
    py::list ret;
    ret.append(smc_conti_change_speed_ratio(ConnectNo, Crd, percent));
    return ret;
}

py::list py_smc_conti_get_run_state(WORD ConnectNo, WORD Crd) {
    py::list ret;
    ret.append(smc_conti_get_run_state(ConnectNo, Crd));
    return ret;
}

py::list py_smc_conti_remain_space(WORD ConnectNo, WORD Crd) {
    py::list ret;
    ret.append(smc_conti_remain_space(ConnectNo, Crd));
    return ret;
}

py::list py_smc_conti_read_current_mark(WORD ConnectNo, WORD Crd) {
    py::list ret;
    ret.append(smc_conti_read_current_mark(ConnectNo, Crd));
    return ret;
}

py::list py_smc_conti_line_unit(WORD ConnectNo, WORD Crd, WORD AxisNum, std::vector<WORD> AxisList, std::vector<double> pPosList, WORD posi_mode, long mark) {
    py::list ret;
    WORD* AxisList_arr = &AxisList[0];
    double* pPosList_arr = &pPosList[0];
    ret.append(smc_conti_line_unit(ConnectNo, Crd, AxisNum, AxisList_arr, pPosList_arr, posi_mode, mark));
    return ret;
}

py::list py_smc_conti_arc_move_center_unit(WORD ConnectNo, WORD Crd, WORD AxisNum, std::vector<WORD> AxisList, std::vector<double> Target_Pos, std::vector<double> Cen_Pos, WORD Arc_Dir, long Circle, WORD posi_mode, long mark) {
    py::list ret;
    WORD* AxisList_arr = &AxisList[0];
    double* Target_Pos_arr = &Target_Pos[0];
    double* Cen_Pos_arr = &Cen_Pos[0];
    ret.append(smc_conti_arc_move_center_unit(ConnectNo, Crd, AxisNum, AxisList_arr, Target_Pos_arr, Cen_Pos_arr, Arc_Dir, Circle, posi_mode, mark));
    return ret;
}

py::list py_smc_conti_arc_move_radius_unit(WORD ConnectNo, WORD Crd, WORD AxisNum, std::vector<WORD> AxisList, std::vector<double> Target_Pos, double Arc_Radius, WORD Arc_Dir, long Circle, WORD posi_mode, long mark) {
    py::list ret;
    WORD* AxisList_arr = &AxisList[0];
    double* Target_Pos_arr = &Target_Pos[0];
    ret.append(smc_conti_arc_move_radius_unit(ConnectNo, Crd, AxisNum, AxisList_arr, Target_Pos_arr, Arc_Radius, Arc_Dir, Circle, posi_mode, mark));
    return ret;
}

py::list py_smc_conti_arc_move_3points_unit(WORD ConnectNo, WORD Crd, WORD AxisNum, std::vector<WORD> AxisList, std::vector<double> Target_Pos, std::vector<double> Mid_Pos, long Circle, WORD posi_mode, long mark) {
    py::list ret;
    WORD* AxisList_arr = &AxisList[0];
    double* Target_Pos_arr = &Target_Pos[0];
    double* Mid_Pos_arr = &Mid_Pos[0];
    ret.append(smc_conti_arc_move_3points_unit(ConnectNo, Crd, AxisNum, AxisList_arr, Target_Pos_arr, Mid_Pos_arr, Circle, posi_mode, mark));
    return ret;
}

py::list py_smc_conti_wait_input(WORD ConnectNo, WORD Crd, WORD bitno, WORD on_off, double TimeOut, long mark) {
    py::list ret;
    ret.append(smc_conti_wait_input(ConnectNo, Crd, bitno, on_off, TimeOut, mark));
    return ret;
}

py::list py_smc_conti_delay_outbit_to_start(WORD ConnectNo, WORD Crd, WORD bitno, WORD on_off, double delay_value, WORD delay_mode, double ReverseTime) {
    py::list ret;
    ret.append(smc_conti_delay_outbit_to_start(ConnectNo, Crd, bitno, on_off, delay_value, delay_mode, ReverseTime));
    return ret;
}

py::list py_smc_conti_delay_outbit_to_stop(WORD ConnectNo, WORD Crd, WORD bitno, WORD on_off, double delay_time, double ReverseTime) {
    py::list ret;
    ret.append(smc_conti_delay_outbit_to_stop(ConnectNo, Crd, bitno, on_off, delay_time, ReverseTime));
    return ret;
}

py::list py_smc_conti_ahead_outbit_to_stop(WORD ConnectNo, WORD Crd, WORD bitno, WORD on_off, double ahead_value, WORD ahead_mode, double ReverseTime) {
    py::list ret;
    ret.append(smc_conti_ahead_outbit_to_stop(ConnectNo, Crd, bitno, on_off, ahead_value, ahead_mode, ReverseTime));
    return ret;
}

py::list py_smc_conti_accurate_outbit_unit(WORD ConnectNo, WORD Crd, WORD cmp_no, WORD on_off, WORD axis, double abs_pos, WORD pos_source, double ReverseTime) {
    py::list ret;
    ret.append(smc_conti_accurate_outbit_unit(ConnectNo, Crd, cmp_no, on_off, axis, abs_pos, pos_source, ReverseTime));
    return ret;
}

py::list py_smc_conti_write_outbit(WORD ConnectNo, WORD Crd, WORD bitno, WORD on_off, double ReverseTime) {
    py::list ret;
    ret.append(smc_conti_write_outbit(ConnectNo, Crd, bitno, on_off, ReverseTime));
    return ret;
}

py::list py_smc_conti_clear_io_action(WORD ConnectNo, WORD Crd, DWORD Io_Mask) {
    py::list ret;
    ret.append(smc_conti_clear_io_action(ConnectNo, Crd, Io_Mask));
    return ret;
}

py::list py_smc_conti_set_pause_output(WORD ConnectNo, WORD Crd, WORD action, long mask, long state) {
    py::list ret;
    ret.append(smc_conti_set_pause_output(ConnectNo, Crd, action, mask, state));
    return ret;
}

py::list py_smc_conti_get_pause_output(WORD ConnectNo, WORD Crd) {
    WORD action = 0;
    long mask = 0;
    long state = 0;
    py::list ret;
    ret.append(smc_conti_get_pause_output(ConnectNo, Crd, &action, &mask, &state));
    ret.append(action);
    ret.append(mask);
    ret.append(state);
    return ret;
}

py::list py_smc_conti_set_override(WORD ConnectNo, WORD Crd, double Percent) {
    py::list ret;
    ret.append(smc_conti_set_override(ConnectNo, Crd, Percent));
    return ret;
}

py::list py_smc_conti_set_blend(WORD ConnectNo, WORD Crd, WORD enable) {
    py::list ret;
    ret.append(smc_conti_set_blend(ConnectNo, Crd, enable));
    return ret;
}

py::list py_smc_conti_get_blend(WORD ConnectNo, WORD Crd) {
    WORD enable = 0;
    py::list ret;
    ret.append(smc_conti_get_blend(ConnectNo, Crd, &enable));
    ret.append(enable);
    return ret;
}

py::list py_smc_conti_pmove_unit(WORD ConnectNo, WORD Crd, WORD axis, double dist, WORD posi_mode, WORD mode, long mark) {
    py::list ret;
    ret.append(smc_conti_pmove_unit(ConnectNo, Crd, axis, dist, posi_mode, mode, mark));
    return ret;
}

py::list py_smc_conti_delay(WORD ConnectNo, WORD Crd, double delay_time, long mark) {
    py::list ret;
    ret.append(smc_conti_delay(ConnectNo, Crd, delay_time, mark));
    return ret;
}

py::list py_smc_set_pwm_enable(WORD ConnectNo, WORD pwmno, WORD enable) {
    py::list ret;
    ret.append(smc_set_pwm_enable(ConnectNo, pwmno, enable));
    return ret;
}

py::list py_smc_get_pwm_enable(WORD ConnectNo, WORD pwmno) {
    WORD enable = 0;
    py::list ret;
    ret.append(smc_get_pwm_enable(ConnectNo, pwmno, &enable));
    ret.append(enable);
    return ret;
}

py::list py_smc_set_pwm_output(WORD ConnectNo, WORD pwmno, double fDuty, double fFre) {
    py::list ret;
    ret.append(smc_set_pwm_output(ConnectNo, pwmno, fDuty, fFre));
    return ret;
}

py::list py_smc_get_pwm_output(WORD ConnectNo, WORD pwmno) {
    double fDuty = 0;
    double fFre = 0;
    py::list ret;
    ret.append(smc_get_pwm_output(ConnectNo, pwmno, &fDuty, &fFre));
    ret.append(fDuty);
    ret.append(fFre);
    return ret;
}

py::list py_smc_conti_set_pwm_output(WORD ConnectNo, WORD Crd, WORD pwmno, double fDuty, double fFre) {
    py::list ret;
    ret.append(smc_conti_set_pwm_output(ConnectNo, Crd, pwmno, fDuty, fFre));
    return ret;
}

py::list py_smc_set_pwm_follow_speed(WORD ConnectNo, WORD pwmno, WORD mode, double MaxVel, double MaxValue, double OutValue) {
    py::list ret;
    ret.append(smc_set_pwm_follow_speed(ConnectNo, pwmno, mode, MaxVel, MaxValue, OutValue));
    return ret;
}

py::list py_smc_get_pwm_follow_speed(WORD ConnectNo, WORD pwmno) {
    WORD mode = 0;
    double MaxVel = 0;
    double MaxValue = 0;
    double OutValue = 0;
    py::list ret;
    ret.append(smc_get_pwm_follow_speed(ConnectNo, pwmno, &mode, &MaxVel, &MaxValue, &OutValue));
    ret.append(mode);
    ret.append(MaxVel);
    ret.append(MaxValue);
    ret.append(OutValue);
    return ret;
}

py::list py_smc_set_pwm_onoff_duty(WORD ConnectNo, WORD pwmno, double fOnDuty, double fOffDuty) {
    py::list ret;
    ret.append(smc_set_pwm_onoff_duty(ConnectNo, pwmno, fOnDuty, fOffDuty));
    return ret;
}

py::list py_smc_get_pwm_onoff_duty(WORD ConnectNo, WORD pwmno) {
    double fOnDuty = 0;
    double fOffDuty = 0;
    py::list ret;
    ret.append(smc_get_pwm_onoff_duty(ConnectNo, pwmno, &fOnDuty, &fOffDuty));
    ret.append(fOnDuty);
    ret.append(fOffDuty);
    return ret;
}

py::list py_smc_set_pwm_follow_onoff(WORD ConnectNo, WORD pwmno, WORD Crd, WORD on_off) {
    py::list ret;
    ret.append(smc_set_pwm_follow_onoff(ConnectNo, pwmno, Crd, on_off));
    return ret;
}

py::list py_smc_get_pwm_follow_onoff(WORD ConnectNo, WORD pwmno) {
    WORD Crd = 0;
    WORD on_off = 0;
    py::list ret;
    ret.append(smc_get_pwm_follow_onoff(ConnectNo, pwmno, &Crd, &on_off));
    ret.append(Crd);
    ret.append(on_off);
    return ret;
}

py::list py_smc_conti_delay_pwm_to_start(WORD ConnectNo, WORD Crd, WORD pwmno, WORD on_off, double delay_value, WORD delay_mode, double ReverseTime) {
    py::list ret;
    ret.append(smc_conti_delay_pwm_to_start(ConnectNo, Crd, pwmno, on_off, delay_value, delay_mode, ReverseTime));
    return ret;
}

py::list py_smc_conti_ahead_pwm_to_stop(WORD ConnectNo, WORD Crd, WORD pwmno, WORD on_off, double ahead_value, WORD ahead_mode, double ReverseTime) {
    py::list ret;
    ret.append(smc_conti_ahead_pwm_to_stop(ConnectNo, Crd, pwmno, on_off, ahead_value, ahead_mode, ReverseTime));
    return ret;
}

py::list py_smc_conti_write_pwm(WORD ConnectNo, WORD Crd, WORD pwmno, WORD on_off, double ReverseTime) {
    py::list ret;
    ret.append(smc_conti_write_pwm(ConnectNo, Crd, pwmno, on_off, ReverseTime));
    return ret;
}

py::list py_smc_laser_set_output(WORD ConnectNo, WORD Enable, WORD Width) {
    py::list ret;
    ret.append(smc_laser_set_output(ConnectNo, Enable, Width));
    return ret;
}

py::list py_smc_laser_get_output(WORD ConnectNo) {
    WORD Enable = 0;
    WORD Width = 0;
    py::list ret;
    ret.append(smc_laser_get_output(ConnectNo, &Enable, &Width));
    ret.append(Enable);
    ret.append(Width);
    return ret;
}

py::list py_smc_set_counter_inmode(WORD ConnectNo, WORD axis, WORD mode) {
    py::list ret;
    ret.append(smc_set_counter_inmode(ConnectNo, axis, mode));
    return ret;
}

py::list py_smc_get_counter_inmode(WORD ConnectNo, WORD axis) {
    WORD mode = 0;
    py::list ret;
    ret.append(smc_get_counter_inmode(ConnectNo, axis, &mode));
    ret.append(mode);
    return ret;
}

py::list py_smc_set_counter_reverse(WORD ConnectNo, WORD axis, WORD reverse) {
    py::list ret;
    ret.append(smc_set_counter_reverse(ConnectNo, axis, reverse));
    return ret;
}

py::list py_smc_get_counter_reverse(WORD ConnectNo, WORD axis) {
    WORD reverse = 0;
    py::list ret;
    ret.append(smc_get_counter_reverse(ConnectNo, axis, &reverse));
    ret.append(reverse);
    return ret;
}

py::list py_smc_set_encoder_unit(WORD ConnectNo, WORD axis, double pos) {
    py::list ret;
    ret.append(smc_set_encoder_unit(ConnectNo, axis, pos));
    return ret;
}

py::list py_smc_get_encoder_unit(WORD ConnectNo, WORD axis) {
    double pos = 0;
    py::list ret;
    ret.append(smc_get_encoder_unit(ConnectNo, axis, &pos));
    ret.append(pos);
    return ret;
}

py::list py_smc_set_extra_encoder_mode(WORD ConnectNo, WORD channel, WORD inmode, WORD multi) {
    py::list ret;
    ret.append(smc_set_extra_encoder_mode(ConnectNo, channel, inmode, multi));
    return ret;
}

py::list py_smc_get_extra_encoder_mode(WORD ConnectNo, WORD channel) {
    WORD inmode = 0;
    WORD multi = 0;
    py::list ret;
    ret.append(smc_get_extra_encoder_mode(ConnectNo, channel, &inmode, &multi));
    ret.append(inmode);
    ret.append(multi);
    return ret;
}

py::list py_smc_set_extra_encoder(WORD ConnectNo, WORD channel, int pos) {
    py::list ret;
    ret.append(smc_set_extra_encoder(ConnectNo, channel, pos));
    return ret;
}

py::list py_smc_get_extra_encoder(WORD ConnectNo, WORD channel) {
    int pos = 0;
    py::list ret;
    ret.append(smc_get_extra_encoder(ConnectNo, channel, &pos));
    ret.append(pos);
    return ret;
}

py::list py_smc_read_inbit(WORD ConnectNo, WORD bitno) {
    py::list ret;
    ret.append(smc_read_inbit(ConnectNo, bitno));
    return ret;
}

py::list py_smc_write_outbit(WORD ConnectNo, WORD bitno, WORD on_off) {
    py::list ret;
    ret.append(smc_write_outbit(ConnectNo, bitno, on_off));
    return ret;
}

py::list py_smc_read_outbit(WORD ConnectNo, WORD bitno) {
    py::list ret;
    ret.append(smc_read_outbit(ConnectNo, bitno));
    return ret;
}

py::list py_smc_read_inport(WORD ConnectNo, WORD portno) {
    py::list ret;
    ret.append(smc_read_inport(ConnectNo, portno));
    return ret;
}

py::list py_smc_read_outport(WORD ConnectNo, WORD portno) {
    py::list ret;
    ret.append(smc_read_outport(ConnectNo, portno));
    return ret;
}

py::list py_smc_write_outport(WORD ConnectNo, WORD portno, DWORD outport_val) {
    py::list ret;
    ret.append(smc_write_outport(ConnectNo, portno, outport_val));
    return ret;
}

py::list py_smc_read_inbit_ex(WORD ConnectNo, WORD bitno) {
    WORD state = 0;
    py::list ret;
    ret.append(smc_read_inbit_ex(ConnectNo, bitno, &state));
    ret.append(state);
    return ret;
}

py::list py_smc_read_outbit_ex(WORD ConnectNo, WORD bitno) {
    WORD state = 0;
    py::list ret;
    ret.append(smc_read_outbit_ex(ConnectNo, bitno, &state));
    ret.append(state);
    return ret;
}

py::list py_smc_read_inport_ex(WORD ConnectNo, WORD portno) {
    DWORD state = 0;
    py::list ret;
    ret.append(smc_read_inport_ex(ConnectNo, portno, &state));
    ret.append(state);
    return ret;
}

py::list py_smc_read_outport_ex(WORD ConnectNo, WORD portno) {
    DWORD state = 0;
    py::list ret;
    ret.append(smc_read_outport_ex(ConnectNo, portno, &state));
    ret.append(state);
    return ret;
}

py::list py_smc_reverse_outbit(WORD ConnectNo, WORD bitno, double reverse_time) {
    py::list ret;
    ret.append(smc_reverse_outbit(ConnectNo, bitno, reverse_time));
    return ret;
}

py::list py_smc_set_outbit_delay_reverse(WORD ConnectNo, WORD channel, WORD outbit, WORD outlevel, double outtime, WORD outmode) {
    py::list ret;
    ret.append(smc_set_outbit_delay_reverse(ConnectNo, channel, outbit, outlevel, outtime, outmode));
    return ret;
}

py::list py_smc_set_io_pwmoutput(WORD ConnectNo, WORD outbit, double time1, double time2, DWORD counts) {
    py::list ret;
    ret.append(smc_set_io_pwmoutput(ConnectNo, outbit, time1, time2, counts));
    return ret;
}

py::list py_smc_clear_io_pwmoutput(WORD ConnectNo, WORD outbit) {
    py::list ret;
    ret.append(smc_clear_io_pwmoutput(ConnectNo, outbit));
    return ret;
}

py::list py_smc_set_io_count_mode(WORD ConnectNo, WORD bitno, WORD mode, double filter) {
    py::list ret;
    ret.append(smc_set_io_count_mode(ConnectNo, bitno, mode, filter));
    return ret;
}

py::list py_smc_get_io_count_mode(WORD ConnectNo, WORD bitno) {
    WORD mode = 0;
    double filter = 0.0;
    py::list ret;
    ret.append(smc_get_io_count_mode(ConnectNo, bitno, &mode, &filter));
    ret.append(mode);
    ret.append(filter);
    return ret;
}

py::list py_smc_set_io_count_value(WORD ConnectNo, WORD bitno, DWORD CountValue) {
    py::list ret;
    ret.append(smc_set_io_count_value(ConnectNo, bitno, CountValue));
    return ret;
}

py::list py_smc_get_io_count_value(WORD ConnectNo, WORD bitno) {
    DWORD CountValue = 0;
    py::list ret;
    ret.append(smc_get_io_count_value(ConnectNo, bitno, &CountValue));
    ret.append(CountValue);
    return ret;
}

py::list py_smc_set_io_map_virtual(WORD ConnectNo, WORD bitno, WORD MapIoType, WORD MapIoIndex, double Filter) {
    py::list ret;
    ret.append(smc_set_io_map_virtual(ConnectNo, bitno, MapIoType, MapIoIndex, Filter));
    return ret;
}

py::list py_smc_get_io_map_virtual(WORD ConnectNo, WORD bitno) {
    WORD MapIoType = 0;
    WORD MapIoIndex = 0;
    double Filter = 0;
    py::list ret;
    ret.append(smc_get_io_map_virtual(ConnectNo, bitno, &MapIoType, &MapIoIndex, &Filter));
    ret.append(MapIoType);
    ret.append(MapIoIndex);
    ret.append(Filter);
    return ret;
}

py::list py_smc_read_inbit_virtual(WORD ConnectNo, WORD bitno) {
    py::list ret;
    ret.append(smc_read_inbit_virtual(ConnectNo, bitno));
    return ret;
}

// py::list py_smc_set_io_dstp_mode(WORD ConnectNo, WORD axis, WORD enable, WORD logic) {
//     py::list ret;
//     ret.append(smc_set_io_dstp_mode(ConnectNo, axis, enable, logic));
//     return ret;
// }

// py::list py_smc_get_io_dstp_mode(WORD ConnectNo, WORD axis, WORD* enable, WORD* logic) {
//     py::list ret;
//     ret.append(smc_get_io_dstp_mode(ConnectNo, axis, enable, logic));
//     return ret;
// }

// py::list py_smc_set_alm_mode(WORD ConnectNo, WORD axis, WORD enable, WORD alm_logic, WORD alm_action) {
//     py::list ret;
//     ret.append(smc_set_alm_mode(ConnectNo, axis, enable, alm_logic, alm_action));
//     return ret;
// }

// py::list py_smc_get_alm_mode(WORD ConnectNo, WORD axis, WORD* enable, WORD* alm_logic, WORD* alm_action) {
//     py::list ret;
//     ret.append(smc_get_alm_mode(ConnectNo, axis, enable, alm_logic, alm_action));
//     return ret;
// }

// py::list py_smc_set_inp_mode(WORD ConnectNo, WORD axis, WORD enable, WORD inp_logic) {
//     py::list ret;
//     ret.append(smc_set_inp_mode(ConnectNo, axis, enable, inp_logic));
//     return ret;
// }

// py::list py_smc_get_inp_mode(WORD ConnectNo, WORD axis, WORD* enable, WORD* inp_logic) {
//     py::list ret;
//     ret.append(smc_get_inp_mode(ConnectNo, axis, enable, inp_logic));
//     return ret;
// }

// py::list py_smc_write_sevon_pin(WORD ConnectNo, WORD axis, WORD on_off) {
//     py::list ret;
//     ret.append(smc_write_sevon_pin(ConnectNo, axis, on_off));
//     return ret;
// }

// py::list py_smc_read_sevon_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_sevon_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_write_erc_pin(WORD ConnectNo, WORD axis, WORD on_off) {
//     py::list ret;
//     ret.append(smc_write_erc_pin(ConnectNo, axis, on_off));
//     return ret;
// }

// py::list py_smc_read_erc_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_erc_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_read_alarm_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_alarm_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_read_inp_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_inp_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_read_org_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_org_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_read_elp_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_elp_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_read_eln_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_eln_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_read_emg_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_emg_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_read_ez_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_ez_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_read_rdy_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_rdy_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_read_cmp_pin(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_read_cmp_pin(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_write_cmp_pin(WORD ConnectNo, WORD axis, WORD on_off) {
//     py::list ret;
//     ret.append(smc_write_cmp_pin(ConnectNo, axis, on_off));
//     return ret;
// }

// py::list py_smc_read_sevon_pin_ex(WORD ConnectNo, WORD axis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_sevon_pin_ex(ConnectNo, axis, state));
//     return ret;
// }

// py::list py_smc_read_erc_pin_ex(WORD ConnectNo, WORD axis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_erc_pin_ex(ConnectNo, axis, state));
//     return ret;
// }

// py::list py_smc_read_alarm_pin_ex(WORD ConnectNo, WORD axis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_alarm_pin_ex(ConnectNo, axis, state));
//     return ret;
// }

// py::list py_smc_read_inp_pin_ex(WORD ConnectNo, WORD axis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_inp_pin_ex(ConnectNo, axis, state));
//     return ret;
// }

// py::list py_smc_read_org_pin_ex(WORD ConnectNo, WORD uiaxis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_org_pin_ex(ConnectNo, uiaxis, state));
//     return ret;
// }

// py::list py_smc_read_elp_pin_ex(WORD ConnectNo, WORD uiaxis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_elp_pin_ex(ConnectNo, uiaxis, state));
//     return ret;
// }

// py::list py_smc_read_eln_pin_ex(WORD ConnectNo, WORD uiaxis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_eln_pin_ex(ConnectNo, uiaxis, state));
//     return ret;
// }

// py::list py_smc_read_emg_pin_ex(WORD ConnectNo, WORD uiaxis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_emg_pin_ex(ConnectNo, uiaxis, state));
//     return ret;
// }

// py::list py_smc_read_ez_pin_ex(WORD ConnectNo, WORD axis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_ez_pin_ex(ConnectNo, axis, state));
//     return ret;
// }

// py::list py_smc_read_rdy_pin_ex(WORD ConnectNo, WORD axis, WORD* state) {
//     py::list ret;
//     ret.append(smc_read_rdy_pin_ex(ConnectNo, axis, state));
//     return ret;
// }

// py::list py_smc_compare_set_config(WORD ConnectNo, WORD axis, WORD enable, WORD cmp_source) {
//     py::list ret;
//     ret.append(smc_compare_set_config(ConnectNo, axis, enable, cmp_source));
//     return ret;
// }

// py::list py_smc_compare_get_config(WORD ConnectNo, WORD axis, WORD* enable, WORD* cmp_source) {
//     py::list ret;
//     ret.append(smc_compare_get_config(ConnectNo, axis, enable, cmp_source));
//     return ret;
// }

// py::list py_smc_compare_clear_points(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_compare_clear_points(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_compare_add_point_unit(WORD ConnectNo, WORD axis, double pos, WORD dir, WORD action, DWORD actpara) {
//     py::list ret;
//     ret.append(smc_compare_add_point_unit(ConnectNo, axis, pos, dir, action, actpara));
//     return ret;
// }

// py::list py_smc_compare_add_point_cycle(WORD ConnectNo, WORD axis, double pos, WORD dir, DWORD bitno, DWORD cycle, WORD level) {
//     py::list ret;
//     ret.append(smc_compare_add_point_cycle(ConnectNo, axis, pos, dir, bitno, cycle, level));
//     return ret;
// }

// py::list py_smc_compare_get_current_point_unit(WORD ConnectNo, WORD axis, double* pos) {
//     py::list ret;
//     ret.append(smc_compare_get_current_point_unit(ConnectNo, axis, pos));
//     return ret;
// }

// py::list py_smc_compare_get_points_runned(WORD ConnectNo, WORD axis, long* pointNum) {
//     py::list ret;
//     ret.append(smc_compare_get_points_runned(ConnectNo, axis, pointNum));
//     return ret;
// }

// py::list py_smc_compare_get_points_remained(WORD ConnectNo, WORD axis, long* pointNum) {
//     py::list ret;
//     ret.append(smc_compare_get_points_remained(ConnectNo, axis, pointNum));
//     return ret;
// }

// py::list py_smc_compare_set_config_extern(WORD ConnectNo, WORD enable, WORD cmp_source) {
//     py::list ret;
//     ret.append(smc_compare_set_config_extern(ConnectNo, enable, cmp_source));
//     return ret;
// }

// py::list py_smc_compare_get_config_extern(WORD ConnectNo, WORD* enable, WORD* cmp_source) {
//     py::list ret;
//     ret.append(smc_compare_get_config_extern(ConnectNo, enable, cmp_source));
//     return ret;
// }

// py::list py_smc_compare_clear_points_extern(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_compare_clear_points_extern(ConnectNo));
//     return ret;
// }

// py::list py_smc_compare_add_point_extern_unit(WORD ConnectNo, WORD* axis, double* pos, WORD* dir, WORD action, DWORD actpara) {
//     py::list ret;
//     ret.append(smc_compare_add_point_extern_unit(ConnectNo, axis, pos, dir, action, actpara));
//     return ret;
// }

// py::list py_smc_compare_add_point_cycle_2d(WORD ConnectNo, WORD* axis, double* pos, WORD* dir, DWORD bitno, DWORD cycle, WORD level) {
//     py::list ret;
//     ret.append(smc_compare_add_point_cycle_2d(ConnectNo, axis, pos, dir, bitno, cycle, level));
//     return ret;
// }

// py::list py_smc_compare_get_current_point_extern_unit(WORD ConnectNo, double* pos) {
//     py::list ret;
//     ret.append(smc_compare_get_current_point_extern_unit(ConnectNo, pos));
//     return ret;
// }

// py::list py_smc_compare_get_points_runned_extern(WORD ConnectNo, long* pointNum) {
//     py::list ret;
//     ret.append(smc_compare_get_points_runned_extern(ConnectNo, pointNum));
//     return ret;
// }

// py::list py_smc_compare_get_points_remained_extern(WORD ConnectNo, long* pointNum) {
//     py::list ret;
//     ret.append(smc_compare_get_points_remained_extern(ConnectNo, pointNum));
//     return ret;
// }

// py::list py_smc_hcmp_set_mode(WORD ConnectNo, WORD hcmp, WORD cmp_mode) {
//     py::list ret;
//     ret.append(smc_hcmp_set_mode(ConnectNo, hcmp, cmp_mode));
//     return ret;
// }

// py::list py_smc_hcmp_get_mode(WORD ConnectNo, WORD hcmp, WORD* cmp_mode) {
//     py::list ret;
//     ret.append(smc_hcmp_get_mode(ConnectNo, hcmp, cmp_mode));
//     return ret;
// }

// py::list py_smc_hcmp_set_config(WORD ConnectNo, WORD hcmp, WORD axis, WORD cmp_source, WORD cmp_logic, long time) {
//     py::list ret;
//     ret.append(smc_hcmp_set_config(ConnectNo, hcmp, axis, cmp_source, cmp_logic, time));
//     return ret;
// }

// py::list py_smc_hcmp_get_config(WORD ConnectNo, WORD hcmp, WORD* axis, WORD* cmp_source, WORD* cmp_logic, long* time) {
//     py::list ret;
//     ret.append(smc_hcmp_get_config(ConnectNo, hcmp, axis, cmp_source, cmp_logic, time));
//     return ret;
// }

// py::list py_smc_hcmp_add_point_unit(WORD ConnectNo, WORD hcmp, double cmp_pos) {
//     py::list ret;
//     ret.append(smc_hcmp_add_point_unit(ConnectNo, hcmp, cmp_pos));
//     return ret;
// }

// py::list py_smc_hcmp_set_liner_unit(WORD ConnectNo, WORD hcmp, double Increment, long Count) {
//     py::list ret;
//     ret.append(smc_hcmp_set_liner_unit(ConnectNo, hcmp, Increment, Count));
//     return ret;
// }

// py::list py_smc_hcmp_get_liner_unit(WORD ConnectNo, WORD hcmp, double* Increment, long* Count) {
//     py::list ret;
//     ret.append(smc_hcmp_get_liner_unit(ConnectNo, hcmp, Increment, Count));
//     return ret;
// }

// py::list py_smc_hcmp_get_current_state_unit(WORD ConnectNo, WORD hcmp, long* remained_points, double* current_point, long* runned_points) {
//     py::list ret;
//     ret.append(smc_hcmp_get_current_state_unit(ConnectNo, hcmp, remained_points, current_point, runned_points));
//     return ret;
// }

// py::list py_smc_hcmp_clear_points(WORD ConnectNo, WORD hcmp) {
//     py::list ret;
//     ret.append(smc_hcmp_clear_points(ConnectNo, hcmp));
//     return ret;
// }

// py::list py_smc_hcmp_2d_set_enable(WORD ConnectNo, WORD hcmp, WORD cmp_enable) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_set_enable(ConnectNo, hcmp, cmp_enable));
//     return ret;
// }

// py::list py_smc_hcmp_2d_get_enable(WORD ConnectNo, WORD hcmp, WORD* cmp_enable) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_get_enable(ConnectNo, hcmp, cmp_enable));
//     return ret;
// }

// py::list py_smc_hcmp_2d_set_config_unit(WORD ConnectNo, WORD hcmp, WORD cmp_mode, WORD x_axis, WORD x_cmp_source, double x_cmp_error, WORD y_axis, WORD y_cmp_source, double y_cmp_error, WORD cmp_logic, int time) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_set_config_unit(ConnectNo, hcmp, cmp_mode, x_axis, x_cmp_source, x_cmp_error, y_axis, y_cmp_source, y_cmp_error, cmp_logic, time));
//     return ret;
// }

// py::list py_smc_hcmp_2d_get_config_unit(WORD ConnectNo, WORD hcmp, WORD* cmp_mode, WORD* x_axis, WORD* x_cmp_source, double* x_cmp_error, WORD* y_axis, WORD* y_cmp_source, double* y_cmp_error, WORD* cmp_logic, int* time) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_get_config_unit(ConnectNo, hcmp, cmp_mode, x_axis, x_cmp_source, x_cmp_error, y_axis, y_cmp_source, y_cmp_error, cmp_logic, time));
//     return ret;
// }

// py::list py_smc_hcmp_2d_set_pwmoutput(WORD ConnectNo, WORD hcmp, WORD pwm_enable, double duty, double freq, WORD pwm_number) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_set_pwmoutput(ConnectNo, hcmp, pwm_enable, duty, freq, pwm_number));
//     return ret;
// }

// py::list py_smc_hcmp_2d_get_pwmoutput(WORD ConnectNo, WORD hcmp, WORD* pwm_enable, double* duty, double* freq, WORD* pwm_number) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_get_pwmoutput(ConnectNo, hcmp, pwm_enable, duty, freq, pwm_number));
//     return ret;
// }

// py::list py_smc_hcmp_2d_add_point_unit(WORD ConnectNo, WORD hcmp, double x_cmp_pos, double y_cmp_pos, WORD cmp_outbit) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_add_point_unit(ConnectNo, hcmp, x_cmp_pos, y_cmp_pos, cmp_outbit));
//     return ret;
// }

// py::list py_smc_hcmp_2d_get_current_state_unit(WORD ConnectNo, WORD hcmp, int* remained_points, double* x_current_point, double* y_current_point, int* runned_points, WORD* current_state, WORD* current_outbit) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_get_current_state_unit(ConnectNo, hcmp, remained_points, x_current_point, y_current_point, runned_points, current_state, current_outbit));
//     return ret;
// }

// py::list py_smc_hcmp_2d_clear_points(WORD ConnectNo, WORD hcmp) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_clear_points(ConnectNo, hcmp));
//     return ret;
// }

// py::list py_smc_hcmp_2d_force_output(WORD ConnectNo, WORD hcmp, WORD outbit) {
//     py::list ret;
//     ret.append(smc_hcmp_2d_force_output(ConnectNo, hcmp, outbit));
//     return ret;
// }

// py::list py_smc_set_homelatch_mode(WORD ConnectNo, WORD axis, WORD enable, WORD logic, WORD source) {
//     py::list ret;
//     ret.append(smc_set_homelatch_mode(ConnectNo, axis, enable, logic, source));
//     return ret;
// }

// py::list py_smc_get_homelatch_mode(WORD ConnectNo, WORD axis, WORD* enable, WORD* logic, WORD* source) {
//     py::list ret;
//     ret.append(smc_get_homelatch_mode(ConnectNo, axis, enable, logic, source));
//     return ret;
// }

// py::list py_smc_get_homelatch_flag(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_get_homelatch_flag(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_reset_homelatch_flag(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_reset_homelatch_flag(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_get_homelatch_value_unit(WORD ConnectNo, WORD axis, double* pos_by_mm) {
//     py::list ret;
//     ret.append(smc_get_homelatch_value_unit(ConnectNo, axis, pos_by_mm));
//     return ret;
// }

// py::list py_smc_set_ezlatch_mode(WORD ConnectNo, WORD axis, WORD enable, WORD logic, WORD source) {
//     py::list ret;
//     ret.append(smc_set_ezlatch_mode(ConnectNo, axis, enable, logic, source));
//     return ret;
// }

// py::list py_smc_get_ezlatch_mode(WORD ConnectNo, WORD axis, WORD* enable, WORD* logic, WORD* source) {
//     py::list ret;
//     ret.append(smc_get_ezlatch_mode(ConnectNo, axis, enable, logic, source));
//     return ret;
// }

// py::list py_smc_get_ezlatch_flag(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_get_ezlatch_flag(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_reset_ezlatch_flag(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_reset_ezlatch_flag(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_get_ezlatch_value_unit(WORD ConnectNo, WORD axis, double* pos_by_mm) {
//     py::list ret;
//     ret.append(smc_get_ezlatch_value_unit(ConnectNo, axis, pos_by_mm));
//     return ret;
// }

// py::list py_smc_set_ltc_mode(WORD ConnectNo, WORD axis, WORD ltc_logic, WORD ltc_mode, double filter) {
//     py::list ret;
//     ret.append(smc_set_ltc_mode(ConnectNo, axis, ltc_logic, ltc_mode, filter));
//     return ret;
// }

// py::list py_smc_get_ltc_mode(WORD ConnectNo, WORD axis, WORD* ltc_logic, WORD* ltc_mode, double* filter) {
//     py::list ret;
//     ret.append(smc_get_ltc_mode(ConnectNo, axis, ltc_logic, ltc_mode, filter));
//     return ret;
// }

// py::list py_smc_set_latch_mode(WORD ConnectNo, WORD axis, WORD all_enable, WORD latch_source, WORD triger_chunnel) {
//     py::list ret;
//     ret.append(smc_set_latch_mode(ConnectNo, axis, all_enable, latch_source, triger_chunnel));
//     return ret;
// }

// py::list py_smc_get_latch_mode(WORD ConnectNo, WORD axis, WORD* all_enable, WORD* latch_source, WORD* triger_chunnel) {
//     py::list ret;
//     ret.append(smc_get_latch_mode(ConnectNo, axis, all_enable, latch_source, triger_chunnel));
//     return ret;
// }

// py::list py_smc_get_latch_flag(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_get_latch_flag(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_reset_latch_flag(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_reset_latch_flag(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_get_latch_value_unit(WORD ConnectNo, WORD axis, double* pos_by_mm) {
//     py::list ret;
//     ret.append(smc_get_latch_value_unit(ConnectNo, axis, pos_by_mm));
//     return ret;
// }

// py::list py_smc_ltc_set_mode(WORD ConnectNo, WORD latch, WORD ltc_mode, WORD ltc_logic, double filter) {
//     py::list ret;
//     ret.append(smc_ltc_set_mode(ConnectNo, latch, ltc_mode, ltc_logic, filter));
//     return ret;
// }

// py::list py_smc_ltc_get_mode(WORD ConnectNo, WORD latch, WORD* ltc_mode, WORD* ltc_logic, double* filter) {
//     py::list ret;
//     ret.append(smc_ltc_get_mode(ConnectNo, latch, ltc_mode, ltc_logic, filter));
//     return ret;
// }

// py::list py_smc_ltc_set_source(WORD ConnectNo, WORD latch, WORD axis, WORD ltc_source) {
//     py::list ret;
//     ret.append(smc_ltc_set_source(ConnectNo, latch, axis, ltc_source));
//     return ret;
// }

// py::list py_smc_ltc_get_source(WORD ConnectNo, WORD latch, WORD axis, WORD* ltc_source) {
//     py::list ret;
//     ret.append(smc_ltc_get_source(ConnectNo, latch, axis, ltc_source));
//     return ret;
// }

// py::list py_smc_ltc_reset(WORD ConnectNo, WORD latch) {
//     py::list ret;
//     ret.append(smc_ltc_reset(ConnectNo, latch));
//     return ret;
// }

// py::list py_smc_ltc_get_number(WORD ConnectNo, WORD latch, WORD axis, int* number) {
//     py::list ret;
//     ret.append(smc_ltc_get_number(ConnectNo, latch, axis, number));
//     return ret;
// }

// py::list py_smc_ltc_get_value_unit(WORD ConnectNo, WORD latch, WORD axis, double* value) {
//     py::list ret;
//     ret.append(smc_ltc_get_value_unit(ConnectNo, latch, axis, value));
//     return ret;
// }

// py::list py_smc_softltc_set_mode(WORD ConnectNo, WORD latch, WORD ltc_enable, WORD ltc_mode, WORD ltc_inbit, WORD ltc_logic, double filter) {
//     py::list ret;
//     ret.append(smc_softltc_set_mode(ConnectNo, latch, ltc_enable, ltc_mode, ltc_inbit, ltc_logic, filter));
//     return ret;
// }

// py::list py_smc_softltc_get_mode(WORD ConnectNo, WORD latch, WORD* ltc_enable, WORD* ltc_mode, WORD* ltc_inbit, WORD* ltc_logic, double* filter) {
//     py::list ret;
//     ret.append(smc_softltc_get_mode(ConnectNo, latch, ltc_enable, ltc_mode, ltc_inbit, ltc_logic, filter));
//     return ret;
// }

// py::list py_smc_softltc_set_source(WORD ConnectNo, WORD latch, WORD axis, WORD ltc_source) {
//     py::list ret;
//     ret.append(smc_softltc_set_source(ConnectNo, latch, axis, ltc_source));
//     return ret;
// }

// py::list py_smc_softltc_get_source(WORD ConnectNo, WORD latch, WORD axis, WORD* ltc_source) {
//     py::list ret;
//     ret.append(smc_softltc_get_source(ConnectNo, latch, axis, ltc_source));
//     return ret;
// }

// py::list py_smc_softltc_reset(WORD ConnectNo, WORD latch) {
//     py::list ret;
//     ret.append(smc_softltc_reset(ConnectNo, latch));
//     return ret;
// }

// py::list py_smc_softltc_get_number(WORD ConnectNo, WORD latch, WORD axis, int* number) {
//     py::list ret;
//     ret.append(smc_softltc_get_number(ConnectNo, latch, axis, number));
//     return ret;
// }

// py::list py_smc_softltc_get_value_unit(WORD ConnectNo, WORD latch, WORD axis, double* value) {
//     py::list ret;
//     ret.append(smc_softltc_get_value_unit(ConnectNo, latch, axis, value));
//     return ret;
// }

// py::list py_smc_set_ain_action(WORD ConnectNo, WORD channel, WORD mode, double fvoltage, WORD action, double actpara) {
//     py::list ret;
//     ret.append(smc_set_ain_action(ConnectNo, channel, mode, fvoltage, action, actpara));
//     return ret;
// }

// py::list py_smc_get_ain_action(WORD ConnectNo, WORD channel, WORD* mode, double* fvoltage, WORD* action, double* actpara) {
//     py::list ret;
//     ret.append(smc_get_ain_action(ConnectNo, channel, mode, fvoltage, action, actpara));
//     return ret;
// }

// py::list py_smc_get_ain_state(WORD ConnectNo, WORD channel) {
//     py::list ret;
//     ret.append(smc_get_ain_state(ConnectNo, channel));
//     return ret;
// }

// py::list py_smc_set_ain_state(WORD ConnectNo, WORD channel) {
//     py::list ret;
//     ret.append(smc_set_ain_state(ConnectNo, channel));
//     return ret;
// }

// py::list py_smc_get_ain(WORD ConnectNo, WORD channel) {
//     py::list ret;
//     ret.append(smc_get_ain(ConnectNo, channel));
//     return ret;
// }

// py::list py_smc_set_da_output(WORD ConnectNo, WORD channel, double Vout) {
//     py::list ret;
//     ret.append(smc_set_da_output(ConnectNo, channel, Vout));
//     return ret;
// }

// py::list py_smc_get_da_output(WORD ConnectNo, WORD channel, double* Vout) {
//     py::list ret;
//     ret.append(smc_get_da_output(ConnectNo, channel, Vout));
//     return ret;
// }

// py::list py_smc_download_file(WORD ConnectNo, const char* pfilename, const char* pfilenameinControl, WORD filetype) {
//     py::list ret;
//     ret.append(smc_download_file(ConnectNo, pfilename, pfilenameinControl, filetype));
//     return ret;
// }

// py::list py_smc_download_memfile(WORD ConnectNo, const char* pbuffer, uint32 buffsize, const char* pfilenameinControl, WORD filetype) {
//     py::list ret;
//     ret.append(smc_download_memfile(ConnectNo, pbuffer, buffsize, pfilenameinControl, filetype));
//     return ret;
// }

// py::list py_smc_upload_file(WORD ConnectNo, const char* pfilename, const char* pfilenameinControl, WORD filetype) {
//     py::list ret;
//     ret.append(smc_upload_file(ConnectNo, pfilename, pfilenameinControl, filetype));
//     return ret;
// }

// py::list py_smc_upload_memfile(WORD ConnectNo, char* pbuffer, uint32 buffsize, const char* pfilenameinControl, uint32* puifilesize, WORD filetype) {
//     py::list ret;
//     ret.append(smc_upload_memfile(ConnectNo, pbuffer, buffsize, pfilenameinControl, puifilesize, filetype));
//     return ret;
// }

// py::list py_smc_download_file_to_ram(WORD ConnectNo, const char* pfilename, WORD filetype) {
//     py::list ret;
//     ret.append(smc_download_file_to_ram(ConnectNo, pfilename, filetype));
//     return ret;
// }

// py::list py_smc_download_memfile_to_ram(WORD ConnectNo, const char* pbuffer, uint32 buffsize, WORD filetype) {
//     py::list ret;
//     ret.append(smc_download_memfile_to_ram(ConnectNo, pbuffer, buffsize, filetype));
//     return ret;
// }

// py::list py_smc_get_progress(WORD ConnectNo, float* process) {
//     py::list ret;
//     ret.append(smc_get_progress(ConnectNo, process));
//     return ret;
// }

// py::list py_smc_udisk_get_state(WORD ConnectNo, WORD* state) {
//     py::list ret;
//     ret.append(smc_udisk_get_state(ConnectNo, state));
//     return ret;
// }

// py::list py_smc_udisk_check_file(WORD ConnectNo, char* filename, int* filesize, WORD filetype) {
//     py::list ret;
//     ret.append(smc_udisk_check_file(ConnectNo, filename, filesize, filetype));
//     return ret;
// }

// py::list py_smc_udisk_get_first_file(WORD ConnectNo, char* filename, int* filesize, int* fileid, WORD filetype) {
//     py::list ret;
//     ret.append(smc_udisk_get_first_file(ConnectNo, filename, filesize, fileid, filetype));
//     return ret;
// }

// py::list py_smc_udisk_get_next_file(WORD ConnectNo, char* filename, int* filesize, int* fileid, WORD filetype) {
//     py::list ret;
//     ret.append(smc_udisk_get_next_file(ConnectNo, filename, filesize, fileid, filetype));
//     return ret;
// }

// py::list py_smc_udisk_copy_file(WORD ConnectNo, const char* SrcFileName, const char* DstFileName, WORD filetype, WORD mode) {
//     py::list ret;
//     ret.append(smc_udisk_copy_file(ConnectNo, SrcFileName, DstFileName, filetype, mode));
//     return ret;
// }

// py::list py_smc_set_modbus_0x(WORD ConnectNo, WORD start, WORD inum, char* pdata) {
//     py::list ret;
//     ret.append(smc_set_modbus_0x(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_get_modbus_0x(WORD ConnectNo, WORD start, WORD inum, char* pdata) {
//     py::list ret;
//     ret.append(smc_get_modbus_0x(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_set_modbus_4x(WORD ConnectNo, WORD start, WORD inum, WORD* pdata) {
//     py::list ret;
//     ret.append(smc_set_modbus_4x(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_get_modbus_4x(WORD ConnectNo, WORD start, WORD inum, WORD* pdata) {
//     py::list ret;
//     ret.append(smc_get_modbus_4x(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_set_modbus_4x_float(WORD ConnectNo, WORD start, WORD inum, const float* pdata) {
//     py::list ret;
//     ret.append(smc_set_modbus_4x_float(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_get_modbus_4x_float(WORD ConnectNo, WORD start, WORD inum, float* pdata) {
//     py::list ret;
//     ret.append(smc_get_modbus_4x_float(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_set_modbus_4x_int(WORD ConnectNo, WORD start, WORD inum, const int* pdata) {
//     py::list ret;
//     ret.append(smc_set_modbus_4x_int(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_get_modbus_4x_int(WORD ConnectNo, WORD start, WORD inum, int* pdata) {
//     py::list ret;
//     ret.append(smc_get_modbus_4x_int(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_set_persistent_reg(WORD ConnectNo, DWORD start, DWORD inum, const char* pdata) {
//     py::list ret;
//     ret.append(smc_set_persistent_reg(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_get_persistent_reg(WORD ConnectNo, DWORD start, DWORD inum, char* pdata) {
//     py::list ret;
//     ret.append(smc_get_persistent_reg(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_set_persistent_reg_byte(WORD ConnectNo, DWORD start, DWORD inum, const char* pdata) {
//     py::list ret;
//     ret.append(smc_set_persistent_reg_byte(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_get_persistent_reg_byte(WORD ConnectNo, DWORD start, DWORD inum, char* pdata) {
//     py::list ret;
//     ret.append(smc_get_persistent_reg_byte(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_set_persistent_reg_float(WORD ConnectNo, DWORD start, DWORD inum, const float* pdata) {
//     py::list ret;
//     ret.append(smc_set_persistent_reg_float(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_get_persistent_reg_float(WORD ConnectNo, DWORD start, DWORD inum, float* pdata) {
//     py::list ret;
//     ret.append(smc_get_persistent_reg_float(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_set_persistent_reg_int(WORD ConnectNo, DWORD start, DWORD inum, const int* pdata) {
//     py::list ret;
//     ret.append(smc_set_persistent_reg_int(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_get_persistent_reg_int(WORD ConnectNo, DWORD start, DWORD inum, int* pdata) {
//     py::list ret;
//     ret.append(smc_get_persistent_reg_int(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_set_persistent_reg_short(WORD ConnectNo, DWORD start, DWORD inum, const short* pdata) {
//     py::list ret;
//     ret.append(smc_set_persistent_reg_short(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_get_persistent_reg_short(WORD ConnectNo, DWORD start, DWORD inum, short* pdata) {
//     py::list ret;
//     ret.append(smc_get_persistent_reg_short(ConnectNo, start, inum, pdata));
//     return ret;
// }

// py::list py_smc_read_array(WORD ConnectNo, const char* name, uint32 index, int64* var, int32* num) {
//     py::list ret;
//     ret.append(smc_read_array(ConnectNo, name, index, var, num));
//     return ret;
// }

// py::list py_smc_modify_array(WORD ConnectNo, const char* name, uint32 index, int64* var, int32 num) {
//     py::list ret;
//     ret.append(smc_modify_array(ConnectNo, name, index, var, num));
//     return ret;
// }

// py::list py_smc_read_var(WORD ConnectNo, const char* varstring, int64* var, int32* num) {
//     py::list ret;
//     ret.append(smc_read_var(ConnectNo, varstring, var, num));
//     return ret;
// }

// py::list py_smc_modify_var(WORD ConnectNo, const char* varstring, int64* var, int32 varnum) {
//     py::list ret;
//     ret.append(smc_modify_var(ConnectNo, varstring, var, varnum));
//     return ret;
// }

// py::list py_smc_write_array(WORD ConnectNo, const char* name, uint32 startindex, int64* var, int32 num) {
//     py::list ret;
//     ret.append(smc_write_array(ConnectNo, name, startindex, var, num));
//     return ret;
// }

// py::list py_smc_read_array_ex(WORD ConnectNo, const char* name, uint32 index, double* var, int32* num) {
//     py::list ret;
//     ret.append(smc_read_array_ex(ConnectNo, name, index, var, num));
//     return ret;
// }

// py::list py_smc_modify_array_ex(WORD ConnectNo, const char* name, uint32 index, double* var, int32 num) {
//     py::list ret;
//     ret.append(smc_modify_array_ex(ConnectNo, name, index, var, num));
//     return ret;
// }

// py::list py_smc_read_var_ex(WORD ConnectNo, const char* varstring, double* var, int32* num) {
//     py::list ret;
//     ret.append(smc_read_var_ex(ConnectNo, varstring, var, num));
//     return ret;
// }

// py::list py_smc_modify_var_ex(WORD ConnectNo, const char* varstring, double* var, int32 varnum) {
//     py::list ret;
//     ret.append(smc_modify_var_ex(ConnectNo, varstring, var, varnum));
//     return ret;
// }

// py::list py_smc_write_array_ex(WORD ConnectNo, const char* name, uint32 startindex, double* var, int32 num) {
//     py::list ret;
//     ret.append(smc_write_array_ex(ConnectNo, name, startindex, var, num));
//     return ret;
// }

// py::list py_smc_get_stringtype(WORD ConnectNo, const char* varstring, int32* m_Type, int32* num) {
//     py::list ret;
//     ret.append(smc_get_stringtype(ConnectNo, varstring, m_Type, num));
//     return ret;
// }

// py::list py_smc_basic_delete_file(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_basic_delete_file(ConnectNo));
//     return ret;
// }

// py::list py_smc_basic_run(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_basic_run(ConnectNo));
//     return ret;
// }

// py::list py_smc_basic_stop(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_basic_stop(ConnectNo));
//     return ret;
// }

// py::list py_smc_basic_pause(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_basic_pause(ConnectNo));
//     return ret;
// }

// py::list py_smc_basic_step_run(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_basic_step_run(ConnectNo));
//     return ret;
// }

// py::list py_smc_basic_step_over(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_basic_step_over(ConnectNo));
//     return ret;
// }

// py::list py_smc_basic_continue_run(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_basic_continue_run(ConnectNo));
//     return ret;
// }

// py::list py_smc_basic_state(WORD ConnectNo, WORD* State) {
//     py::list ret;
//     ret.append(smc_basic_state(ConnectNo, State));
//     return ret;
// }

// py::list py_smc_basic_current_line(WORD ConnectNo, uint32* line) {
//     py::list ret;
//     ret.append(smc_basic_current_line(ConnectNo, line));
//     return ret;
// }

// py::list py_smc_basic_break_info(WORD ConnectNo, uint32* line, uint32 linenum) {
//     py::list ret;
//     ret.append(smc_basic_break_info(ConnectNo, line, linenum));
//     return ret;
// }

// py::list py_smc_basic_message(WORD ConnectNo, char* pbuff, uint32 uimax, uint32* puiread) {
//     py::list ret;
//     ret.append(smc_basic_message(ConnectNo, pbuff, uimax, puiread));
//     return ret;
// }

// py::list py_smc_basic_command(WORD ConnectNo, const char* pszCommand, char* psResponse, uint32 uiResponseLength) {
//     py::list ret;
//     ret.append(smc_basic_command(ConnectNo, pszCommand, psResponse, uiResponseLength));
//     return ret;
// }

// py::list py_smc_gcode_check_file(WORD ConnectNo, const char* pfilenameinControl, uint8* pbIfExist, uint32* pFileSize) {
//     py::list ret;
//     ret.append(smc_gcode_check_file(ConnectNo, pfilenameinControl, pbIfExist, pFileSize));
//     return ret;
// }

// py::list py_smc_gcode_get_first_file(WORD ConnectNo, char* pfilenameinControl, uint32* pFileSize) {
//     py::list ret;
//     ret.append(smc_gcode_get_first_file(ConnectNo, pfilenameinControl, pFileSize));
//     return ret;
// }

// py::list py_smc_gcode_get_next_file(WORD ConnectNo, char* pfilenameinControl, uint32* pFileSize) {
//     py::list ret;
//     ret.append(smc_gcode_get_next_file(ConnectNo, pfilenameinControl, pFileSize));
//     return ret;
// }

// py::list py_smc_gcode_start(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_gcode_start(ConnectNo));
//     return ret;
// }

// py::list py_smc_gcode_stop(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_gcode_stop(ConnectNo));
//     return ret;
// }

// py::list py_smc_gcode_pause(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_gcode_pause(ConnectNo));
//     return ret;
// }

// py::list py_smc_gcode_state(WORD ConnectNo, WORD* State) {
//     py::list ret;
//     ret.append(smc_gcode_state(ConnectNo, State));
//     return ret;
// }

// py::list py_smc_gcode_set_current_file(WORD ConnectNo, const char* pFileName) {
//     py::list ret;
//     ret.append(smc_gcode_set_current_file(ConnectNo, pFileName));
//     return ret;
// }

// py::list py_smc_gcode_get_current_file(WORD ConnectNo, char* pfilenameinControl, WORD* fileid) {
//     py::list ret;
//     ret.append(smc_gcode_get_current_file(ConnectNo, pfilenameinControl, fileid));
//     return ret;
// }

// py::list py_smc_gcode_current_line(WORD ConnectNo, uint32* line, char* pCurLine) {
//     py::list ret;
//     ret.append(smc_gcode_current_line(ConnectNo, line, pCurLine));
//     return ret;
// }

// py::list py_smc_gcode_get_current_line(WORD ConnectNo, uint32* line, char* pCurLine) {
//     py::list ret;
//     ret.append(smc_gcode_get_current_line(ConnectNo, line, pCurLine));
//     return ret;
// }

// py::list py_smc_gcode_check_file_id(WORD ConnectNo, WORD fileid, char* pFileName, uint32* pFileSize, uint32* pTotalLine) {
//     py::list ret;
//     ret.append(smc_gcode_check_file_id(ConnectNo, fileid, pFileName, pFileSize, pTotalLine));
//     return ret;
// }

// py::list py_smc_gcode_check_file_name(WORD ConnectNo, const char* pFileName, WORD* fileid, uint32* pFileSize, uint32* pTotalLine) {
//     py::list ret;
//     ret.append(smc_gcode_check_file_name(ConnectNo, pFileName, fileid, pFileSize, pTotalLine));
//     return ret;
// }

// py::list py_smc_gcode_get_file_profile(WORD ConnectNo, uint32* maxfilenum, uint32* maxfilesize, uint32* savedfilenum) {
//     py::list ret;
//     ret.append(smc_gcode_get_file_profile(ConnectNo, maxfilenum, maxfilesize, savedfilenum));
//     return ret;
// }

// py::list py_smc_gcode_add_line(WORD ConnectNo, const char* strline) {
//     py::list ret;
//     ret.append(smc_gcode_add_line(ConnectNo, strline));
//     return ret;
// }

// py::list py_smc_gcode_add_line_array(WORD ConnectNo, int arraysize, const float* linearray) {
//     py::list ret;
//     ret.append(smc_gcode_add_line_array(ConnectNo, arraysize, linearray));
//     return ret;
// }

// py::list py_smc_gcode_insert_line(WORD ConnectNo, int lineno, const char* strline) {
//     py::list ret;
//     ret.append(smc_gcode_insert_line(ConnectNo, lineno, strline));
//     return ret;
// }

// py::list py_smc_gcode_insert_line_array(WORD ConnectNo, int lineno, int arraysize, const float* linearray) {
//     py::list ret;
//     ret.append(smc_gcode_insert_line_array(ConnectNo, lineno, arraysize, linearray));
//     return ret;
// }

// py::list py_smc_gcode_modify_line(WORD ConnectNo, int lineno, const char* strline) {
//     py::list ret;
//     ret.append(smc_gcode_modify_line(ConnectNo, lineno, strline));
//     return ret;
// }

// py::list py_smc_gcode_modify_line_array(WORD ConnectNo, int lineno, int arraysize, const float* linearray) {
//     py::list ret;
//     ret.append(smc_gcode_modify_line_array(ConnectNo, lineno, arraysize, linearray));
//     return ret;
// }

// py::list py_smc_gcode_delete_line(WORD ConnectNo, int lineno) {
//     py::list ret;
//     ret.append(smc_gcode_delete_line(ConnectNo, lineno));
//     return ret;
// }

// py::list py_smc_gcode_get_line(WORD ConnectNo, uint32 line, char* strLine) {
//     py::list ret;
//     ret.append(smc_gcode_get_line(ConnectNo, line, strLine));
//     return ret;
// }

// py::list py_smc_gcode_get_line_array(WORD ConnectNo, int lineno, int* arraysize, float* linearray) {
//     py::list ret;
//     ret.append(smc_gcode_get_line_array(ConnectNo, lineno, arraysize, linearray));
//     return ret;
// }

// py::list py_smc_gcode_create_file(WORD ConnectNo, const char* FileName) {
//     py::list ret;
//     ret.append(smc_gcode_create_file(ConnectNo, FileName));
//     return ret;
// }

// py::list py_smc_gcode_save_file(WORD ConnectNo, const char* FileName) {
//     py::list ret;
//     ret.append(smc_gcode_save_file(ConnectNo, FileName));
//     return ret;
// }

// py::list py_smc_gcode_copy_file(WORD ConnectNo, const char* strFileName, const char* newFileName) {
//     py::list ret;
//     ret.append(smc_gcode_copy_file(ConnectNo, strFileName, newFileName));
//     return ret;
// }

// py::list py_smc_gcode_rename_file(WORD ConnectNo, const char* strFileName, const char* newFileName) {
//     py::list ret;
//     ret.append(smc_gcode_rename_file(ConnectNo, strFileName, newFileName));
//     return ret;
// }

// py::list py_smc_gcode_delete_fileid(WORD ConnectNo, int fileid) {
//     py::list ret;
//     ret.append(smc_gcode_delete_fileid(ConnectNo, fileid));
//     return ret;
// }

// py::list py_smc_gcode_delete_file(WORD ConnectNo, const char* pfilenameinControl) {
//     py::list ret;
//     ret.append(smc_gcode_delete_file(ConnectNo, pfilenameinControl));
//     return ret;
// }

// py::list py_smc_gcode_clear_file(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_gcode_clear_file(ConnectNo));
//     return ret;
// }

// py::list py_smc_gcode_get_fileid(WORD ConnectNo, uint32 fileid, char* pFileName, uint32* filesize) {
//     py::list ret;
//     ret.append(smc_gcode_get_fileid(ConnectNo, fileid, pFileName, filesize));
//     return ret;
// }

// py::list py_smc_gcode_set_step_state(WORD ConnectNo, WORD state) {
//     py::list ret;
//     ret.append(smc_gcode_set_step_state(ConnectNo, state));
//     return ret;
// }

// py::list py_smc_gcode_get_step_state(WORD ConnectNo, WORD* state) {
//     py::list ret;
//     ret.append(smc_gcode_get_step_state(ConnectNo, state));
//     return ret;
// }

// py::list py_smc_gcode_stop_reason(WORD ConnectNo, WORD* stop_reason) {
//     py::list ret;
//     ret.append(smc_gcode_stop_reason(ConnectNo, stop_reason));
//     return ret;
// }

py::list py_smc_emg_stop(WORD ConnectNo) {
    py::list ret;
    ret.append(smc_emg_stop(ConnectNo));
    return ret;
}

py::list py_smc_check_done(WORD ConnectNo, WORD axis) {
    py::list ret;
    ret.append(smc_check_done(ConnectNo, axis));
    return ret;
}

py::list py_smc_stop(WORD ConnectNo, WORD axis, WORD stop_mode) {
    py::list ret;
    ret.append(smc_stop(ConnectNo, axis, stop_mode));
    return ret;
}

py::list py_smc_check_done_multicoor(WORD ConnectNo, WORD Crd) {
    py::list ret;
    ret.append(smc_check_done_multicoor(ConnectNo, Crd));
    return ret;
}

py::list py_smc_stop_multicoor(WORD ConnectNo, WORD Crd, WORD stop_mode) {
    py::list ret;
    ret.append(smc_stop_multicoor(ConnectNo, Crd, stop_mode));
    return ret;
}

py::list py_smc_axis_io_status(WORD ConnectNo, WORD axis) {
    py::list ret;
    ret.append(smc_axis_io_status(ConnectNo, axis));
    return ret;
}

// py::list py_smc_axis_io_enable_status(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_axis_io_enable_status(ConnectNo, axis));
//     return ret;
// }

py::list py_smc_get_axis_run_mode(WORD ConnectNo, WORD axis) {
    WORD run_mode = 0;
    py::list ret;
    ret.append(smc_get_axis_run_mode(ConnectNo, axis, &run_mode));
    ret.append(run_mode);
    return ret;
}

py::list py_smc_read_current_speed_unit(WORD ConnectNo, WORD axis) {
    double current_speed = 0.0;
    py::list ret;
    ret.append(smc_read_current_speed_unit(ConnectNo, axis, &current_speed));
    ret.append(current_speed);
    return ret;
}

// py::list py_smc_set_position_unit(WORD ConnectNo, WORD axis, double pos) {
//     py::list ret;
//     ret.append(smc_set_position_unit(ConnectNo, axis, pos));
//     return ret;
// }

py::list py_smc_get_position_unit(WORD ConnectNo, WORD axis) {
    double pose = 0.0;
    py::list ret;
    ret.append(smc_get_position_unit(ConnectNo, axis, &pose));
    ret.append(pose);
    return ret;
}

// py::list py_smc_get_target_position_unit(WORD ConnectNo, WORD axis, double* pos) {
//     py::list ret;
//     ret.append(smc_get_target_position_unit(ConnectNo, axis, pos));
//     return ret;
// }

// py::list py_smc_set_workpos_unit(WORD ConnectNo, WORD axis, double pos) {
//     py::list ret;
//     ret.append(smc_set_workpos_unit(ConnectNo, axis, pos));
//     return ret;
// }

// py::list py_smc_get_workpos_unit(WORD ConnectNo, WORD axis, double* pos) {
//     py::list ret;
//     ret.append(smc_get_workpos_unit(ConnectNo, axis, pos));
//     return ret;
// }

// py::list py_smc_get_stop_reason(WORD ConnectNo, WORD axis, long* StopReason) {
//     py::list ret;
//     ret.append(smc_get_stop_reason(ConnectNo, axis, StopReason));
//     return ret;
// }

py::list py_smc_clear_stop_reason(WORD ConnectNo, WORD axis) {
    py::list ret;
    ret.append(smc_clear_stop_reason(ConnectNo, axis));
    return ret;
}

// py::list py_smc_trace_set_source(WORD ConnectNo, WORD source) {
//     py::list ret;
//     ret.append(smc_trace_set_source(ConnectNo, source));
//     return ret;
// }

// py::list py_smc_read_trace_data(WORD ConnectNo, WORD axis, long bufsize, double* time, double* pos, double* vel, double* acc, long* recv_num) {
//     py::list ret;
//     ret.append(smc_read_trace_data(ConnectNo, axis, bufsize, time, pos, vel, acc, recv_num));
//     return ret;
// }

// py::list py_smc_trace_start(WORD ConnectNo, WORD AxisNum, WORD* AxisList) {
//     py::list ret;
//     ret.append(smc_trace_start(ConnectNo, AxisNum, AxisList));
//     return ret;
// }

// py::list py_smc_trace_stop(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_trace_stop(ConnectNo));
//     return ret;
// }

// py::list py_smc_trace_set_config(WORD ConnectNo, short trace_cycle, short lost_handle, short trace_type, short trigger_object_index, short trigger_type, int mask, long long condition) {
//     py::list ret;
//     ret.append(smc_trace_set_config(ConnectNo, trace_cycle, lost_handle, trace_type, trigger_object_index, trigger_type, mask, condition));
//     return ret;
// }

// py::list py_smc_trace_get_config(WORD ConnectNo, short* trace_cycle, short* lost_handle, short* trace_type, short* trigger_object_index, short* trigger_type, int* mask, long long* condition) {
//     py::list ret;
//     ret.append(smc_trace_get_config(ConnectNo, trace_cycle, lost_handle, trace_type, trigger_object_index, trigger_type, mask, condition));
//     return ret;
// }

// py::list py_smc_trace_reset_config_object(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_trace_reset_config_object(ConnectNo));
//     return ret;
// }

// py::list py_smc_trace_add_config_object(WORD ConnectNo, short data_type, short data_index, short data_sub_index, short slave_id, short data_bytes) {
//     py::list ret;
//     ret.append(smc_trace_add_config_object(ConnectNo, data_type, data_index, data_sub_index, slave_id, data_bytes));
//     return ret;
// }

// py::list py_smc_trace_get_config_object(WORD ConnectNo, short object_index, short* data_type, short* data_index, short* data_sub_index, short* slave_id, short* data_bytes) {
//     py::list ret;
//     ret.append(smc_trace_get_config_object(ConnectNo, object_index, data_type, data_index, data_sub_index, slave_id, data_bytes));
//     return ret;
// }

// py::list py_smc_trace_data_start(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_trace_data_start(ConnectNo));
//     return ret;
// }

// py::list py_smc_trace_data_stop(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_trace_data_stop(ConnectNo));
//     return ret;
// }

// py::list py_smc_trace_data_reset(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_trace_data_reset(ConnectNo));
//     return ret;
// }

// py::list py_smc_trace_get_flag(WORD ConnectNo, short* start_flag, short* triggered_flag, short* lost_flag) {
//     py::list ret;
//     ret.append(smc_trace_get_flag(ConnectNo, start_flag, triggered_flag, lost_flag));
//     return ret;
// }

// py::list py_smc_trace_get_state(WORD ConnectNo, int* valid_num, int* free_num, int* object_total_bytes, int* object_total_num) {
//     py::list ret;
//     ret.append(smc_trace_get_state(ConnectNo, valid_num, free_num, object_total_bytes, object_total_num));
//     return ret;
// }

// py::list py_smc_trace_get_data(WORD ConnectNo, int bufsize, unsigned char* data, int* byte_size) {
//     py::list ret;
//     ret.append(smc_trace_get_data(ConnectNo, bufsize, data, byte_size));
//     return ret;
// }

// py::list py_smc_trace_reset_lost_flag(WORD ConnectNo) {
//     py::list ret;
//     ret.append(smc_trace_reset_lost_flag(ConnectNo));
//     return ret;
// }

// py::list py_nmcs_set_node_od(WORD ConnectNo, WORD PortNo, WORD NodeNo, WORD Index, WORD SubIndex, WORD ValLength, DWORD Value) {
//     py::list ret;
//     ret.append(nmcs_set_node_od(ConnectNo, PortNo, NodeNo, Index, SubIndex, ValLength, Value));
//     return ret;
// }

// py::list py_nmcs_get_node_od(WORD ConnectNo, WORD PortNo, WORD NodeNo, WORD Index, WORD SubIndex, WORD ValLength, DWORD* Value) {
//     py::list ret;
//     ret.append(nmcs_get_node_od(ConnectNo, PortNo, NodeNo, Index, SubIndex, ValLength, Value));
//     return ret;
// }

// py::list py_nmcs_set_node_od_float(WORD ConnectNo, WORD PortNum, WORD NodeNum, WORD Index, WORD SubIndex, WORD ValLength, float Value) {
//     py::list ret;
//     ret.append(nmcs_set_node_od_float(ConnectNo, PortNum, NodeNum, Index, SubIndex, ValLength, Value));
//     return ret;
// }

// py::list py_nmcs_get_node_od_float(WORD ConnectNo, WORD PortNum, WORD NodeNum, WORD Index, WORD SubIndex, WORD ValLength, float* Value) {
//     py::list ret;
//     ret.append(nmcs_get_node_od_float(ConnectNo, PortNum, NodeNum, Index, SubIndex, ValLength, Value));
//     return ret;
// }

// py::list py_nmcs_set_node_od_pbyte(WORD ConnectNo, WORD PortNum, WORD NodeNum, WORD Index, WORD SubIndex, WORD Bytes, unsigned char* Value) {
//     py::list ret;
//     ret.append(nmcs_set_node_od_pbyte(ConnectNo, PortNum, NodeNum, Index, SubIndex, Bytes, Value));
//     return ret;
// }

// py::list py_nmcs_get_node_od_pbyte(WORD ConnectNo, WORD PortNum, WORD NodeNum, WORD Index, WORD SubIndex, WORD Bytes, unsigned char* Value) {
//     py::list ret;
//     ret.append(nmcs_get_node_od_pbyte(ConnectNo, PortNum, NodeNum, Index, SubIndex, Bytes, Value));
//     return ret;
// }

// py::list py_nmcs_set_axis_enable(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(nmcs_set_axis_enable(ConnectNo, axis));
//     return ret;
// }

// py::list py_nmcs_set_axis_disable(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(nmcs_set_axis_disable(ConnectNo, axis));
//     return ret;
// }

// py::list py_nmcs_get_axis_io_out(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(nmcs_get_axis_io_out(ConnectNo, axis));
//     return ret;
// }

// py::list py_nmcs_set_axis_io_out(WORD ConnectNo, WORD axis, DWORD iostate) {
//     py::list ret;
//     ret.append(nmcs_set_axis_io_out(ConnectNo, axis, iostate));
//     return ret;
// }

// py::list py_nmcs_get_axis_io_in(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(nmcs_get_axis_io_in(ConnectNo, axis));
//     return ret;
// }

// py::list py_nmcs_set_cycletime(WORD ConnectNo, WORD PortNo, DWORD CycleTime) {
//     py::list ret;
//     ret.append(nmcs_set_cycletime(ConnectNo, PortNo, CycleTime));
//     return ret;
// }

// py::list py_nmcs_get_cycletime(WORD ConnectNo, WORD PortNo, DWORD* CycleTime) {
//     py::list ret;
//     ret.append(nmcs_get_cycletime(ConnectNo, PortNo, CycleTime));
//     return ret;
// }

// py::list py_nmcs_set_offset_pos(WORD ConnectNo, WORD axis, double offset_pos) {
//     py::list ret;
//     ret.append(nmcs_set_offset_pos(ConnectNo, axis, offset_pos));
//     return ret;
// }

// py::list py_nmcs_get_offset_pos(WORD ConnectNo, WORD axis, double* offset_pos) {
//     py::list ret;
//     ret.append(nmcs_get_offset_pos(ConnectNo, axis, offset_pos));
//     return ret;
// }

// py::list py_nmcs_get_axis_type(WORD ConnectNo, WORD axis, WORD* Axis_Type) {
//     py::list ret;
//     ret.append(nmcs_get_axis_type(ConnectNo, axis, Axis_Type));
//     return ret;
// }

// py::list py_nmcs_axis_io_status(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(nmcs_axis_io_status(ConnectNo, axis));
//     return ret;
// }

// py::list py_nmcs_get_card_errcode(WORD ConnectNo, DWORD* Errcode) {
//     py::list ret;
//     ret.append(nmcs_get_card_errcode(ConnectNo, Errcode));
//     return ret;
// }

// py::list py_nmcs_clear_card_errcode(WORD ConnectNo) {
//     py::list ret;
//     ret.append(nmcs_clear_card_errcode(ConnectNo));
//     return ret;
// }

// py::list py_nmcs_get_errcode(WORD ConnectNo, WORD PortNo, DWORD* Errcode) {
//     py::list ret;
//     ret.append(nmcs_get_errcode(ConnectNo, PortNo, Errcode));
//     return ret;
// }

// py::list py_nmcs_clear_errcode(WORD ConnectNo, WORD PortNo) {
//     py::list ret;
//     ret.append(nmcs_clear_errcode(ConnectNo, PortNo));
//     return ret;
// }

// py::list py_nmcs_get_axis_errcode(WORD ConnectNo, WORD axis, DWORD* Errcode) {
//     py::list ret;
//     ret.append(nmcs_get_axis_errcode(ConnectNo, axis, Errcode));
//     return ret;
// }

// py::list py_nmcs_clear_axis_errcode(WORD ConnectNo, WORD iaxis) {
//     py::list ret;
//     ret.append(nmcs_clear_axis_errcode(ConnectNo, iaxis));
//     return ret;
// }

// py::list py_nmcs_get_total_axes(WORD ConnectNo, DWORD* TotalAxis) {
//     py::list ret;
//     ret.append(nmcs_get_total_axes(ConnectNo, TotalAxis));
//     return ret;
// }

// py::list py_nmcs_get_total_ionum(WORD ConnectNo, WORD* TotalIn, WORD* TotalOut) {
//     py::list ret;
//     ret.append(nmcs_get_total_ionum(ConnectNo, TotalIn, TotalOut));
//     return ret;
// }

// py::list py_nmcs_read_inbit_extern(WORD ConnectNo, WORD PortNo, WORD NodeID, WORD BitNo, WORD* IoValue) {
//     py::list ret;
//     ret.append(nmcs_read_inbit_extern(ConnectNo, PortNo, NodeID, BitNo, IoValue));
//     return ret;
// }

// py::list py_nmcs_read_inport_extern(WORD ConnectNo, WORD PortNo, WORD NodeID, WORD IoPortNo, DWORD* IoValue) {
//     py::list ret;
//     ret.append(nmcs_read_inport_extern(ConnectNo, PortNo, NodeID, IoPortNo, IoValue));
//     return ret;
// }

// py::list py_nmcs_write_outbit_extern(WORD ConnectNo, WORD PortNo, WORD NodeID, WORD BitNo, WORD IoValue) {
//     py::list ret;
//     ret.append(nmcs_write_outbit_extern(ConnectNo, PortNo, NodeID, BitNo, IoValue));
//     return ret;
// }

// py::list py_nmcs_write_outport_extern(WORD ConnectNo, WORD PortNo, WORD NodeID, WORD IoPortNo, DWORD IoValue) {
//     py::list ret;
//     ret.append(nmcs_write_outport_extern(ConnectNo, PortNo, NodeID, IoPortNo, IoValue));
//     return ret;
// }

// py::list py_nmcs_read_outbit_extern(WORD ConnectNo, WORD PortNo, WORD NodeID, WORD BitNo, WORD* IoValue) {
//     py::list ret;
//     ret.append(nmcs_read_outbit_extern(ConnectNo, PortNo, NodeID, BitNo, IoValue));
//     return ret;
// }

// py::list py_nmcs_read_outport_extern(WORD ConnectNo, WORD PortNo, WORD NodeID, WORD IoPortNo, DWORD* IoValue) {
//     py::list ret;
//     ret.append(nmcs_read_outport_extern(ConnectNo, PortNo, NodeID, IoPortNo, IoValue));
//     return ret;
// }

// py::list py_nmcs_set_slave_output_retain(WORD ConnectNo, WORD Enable) {
//     py::list ret;
//     ret.append(nmcs_set_slave_output_retain(ConnectNo, Enable));
//     return ret;
// }

// py::list py_nmcs_get_slave_output_retain(WORD ConnectNo, WORD* Enable) {
//     py::list ret;
//     ret.append(nmcs_get_slave_output_retain(ConnectNo, Enable));
//     return ret;
// }

// py::list py_nmcs_reset_canopen(WORD ConnectNo) {
//     py::list ret;
//     ret.append(nmcs_reset_canopen(ConnectNo));
//     return ret;
// }

// py::list py_nmcs_get_LostHeartbeat_Nodes(WORD ConnectNo, WORD PortNo, WORD* NodeID, WORD* NodeNum) {
//     py::list ret;
//     ret.append(nmcs_get_LostHeartbeat_Nodes(ConnectNo, PortNo, NodeID, NodeNum));
//     return ret;
// }

// py::list py_nmcs_get_EmergeneyMessege_Nodes(WORD ConnectNo, WORD PortNo, DWORD* NodeMsg, WORD* MsgNum) {
//     py::list ret;
//     ret.append(nmcs_get_EmergeneyMessege_Nodes(ConnectNo, PortNo, NodeMsg, MsgNum));
//     return ret;
// }

// py::list py_nmcs_SendNmtCommand(WORD ConnectNo, WORD PortNo, WORD NodeID, WORD NmtCommand) {
//     py::list ret;
//     ret.append(nmcs_SendNmtCommand(ConnectNo, PortNo, NodeID, NmtCommand));
//     return ret;
// }

// py::list py_nmcs_set_alarm_clear(WORD ConnectNo, WORD PortNo, WORD NodeNo) {
//     py::list ret;
//     ret.append(nmcs_set_alarm_clear(ConnectNo, PortNo, NodeNo));
//     return ret;
// }

// py::list py_nmcs_syn_move_unit(WORD ConnectNo, WORD AxisNum, WORD* AxisList, double* Position, WORD* PosiMode) {
//     py::list ret;
//     ret.append(nmcs_syn_move_unit(ConnectNo, AxisNum, AxisList, Position, PosiMode));
//     return ret;
// }

// py::list py_nmcs_get_total_adcnum(WORD ConnectNo, WORD* TotalIn, WORD* TotalOut) {
//     py::list ret;
//     ret.append(nmcs_get_total_adcnum(ConnectNo, TotalIn, TotalOut));
//     return ret;
// }

// py::list py_nmcs_reset_etc(WORD ConnectNo) {
//     py::list ret;
//     ret.append(nmcs_reset_etc(ConnectNo));
//     return ret;
// }

// py::list py_nmcs_stop_etc(WORD ConnectNo, WORD* ETCState) {
//     py::list ret;
//     ret.append(nmcs_stop_etc(ConnectNo, ETCState));
//     return ret;
// }

// py::list py_nmcs_get_axis_state_machine(WORD ConnectNo, WORD axis, WORD* Axis_StateMachine) {
//     py::list ret;
//     ret.append(nmcs_get_axis_state_machine(ConnectNo, axis, Axis_StateMachine));
//     return ret;
// }

// py::list py_nmcs_get_axis_node_address(WORD ConnectNo, WORD axis, WORD* SlaveAddr, WORD* Sub_SlaveAddr) {
//     py::list ret;
//     ret.append(nmcs_get_axis_node_address(ConnectNo, axis, SlaveAddr, Sub_SlaveAddr));
//     return ret;
// }

// py::list py_nmcs_write_rxpdo_extra(WORD ConnectNo, WORD PortNo, WORD address, WORD DataLen, DWORD Value) {
//     py::list ret;
//     ret.append(nmcs_write_rxpdo_extra(ConnectNo, PortNo, address, DataLen, Value));
//     return ret;
// }

// py::list py_nmcs_read_rxpdo_extra(WORD ConnectNo, WORD PortNo, WORD address, WORD DataLen, DWORD* Value) {
//     py::list ret;
//     ret.append(nmcs_read_rxpdo_extra(ConnectNo, PortNo, address, DataLen, Value));
//     return ret;
// }

// py::list py_nmcs_read_txpdo_extra(WORD ConnectNo, WORD PortNo, WORD address, WORD DataLen, DWORD* Value) {
//     py::list ret;
//     ret.append(nmcs_read_txpdo_extra(ConnectNo, PortNo, address, DataLen, Value));
//     return ret;
// }

// py::list py_nmcs_torque_move(WORD CardNo, WORD axis, int Torque, WORD PosLimitValid, double PosLimitValue, WORD PosMode) {
//     py::list ret;
//     ret.append(nmcs_torque_move(CardNo, axis, Torque, PosLimitValid, PosLimitValue, PosMode));
//     return ret;
// }

// py::list py_nmcs_change_torque(WORD CardNo, WORD axis, int Torque) {
//     py::list ret;
//     ret.append(nmcs_change_torque(CardNo, axis, Torque));
//     return ret;
// }

// py::list py_nmcs_get_torque(WORD CardNo, WORD axis, int* Torque) {
//     py::list ret;
//     ret.append(nmcs_get_torque(CardNo, axis, Torque));
//     return ret;
// }

// py::list py_smc_pdo_buffer_enter(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_pdo_buffer_enter(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_pdo_buffer_stop(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_pdo_buffer_stop(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_pdo_buffer_clear(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(smc_pdo_buffer_clear(ConnectNo, axis));
//     return ret;
// }

// py::list py_smc_pdo_buffer_run_state(WORD ConnectNo, WORD axis, int* RunState, int* Remain, int* NotRunned, int* Runned) {
//     py::list ret;
//     ret.append(smc_pdo_buffer_run_state(ConnectNo, axis, RunState, Remain, NotRunned, Runned));
//     return ret;
// }

// py::list py_smc_pdo_buffer_add_data(WORD ConnectNo, WORD axis, int size, int* data_table) {
//     py::list ret;
//     ret.append(smc_pdo_buffer_add_data(ConnectNo, axis, size, data_table));
//     return ret;
// }

// py::list py_smc_pdo_buffer_start_multi(WORD ConnectNo, WORD AxisNum, WORD* AxisList, WORD* ResultList) {
//     py::list ret;
//     ret.append(smc_pdo_buffer_start_multi(ConnectNo, AxisNum, AxisList, ResultList));
//     return ret;
// }

// py::list py_smc_pdo_buffer_pause_multi(WORD ConnectNo, WORD AxisNum, WORD* AxisList, WORD* ResultList) {
//     py::list ret;
//     ret.append(smc_pdo_buffer_pause_multi(ConnectNo, AxisNum, AxisList, ResultList));
//     return ret;
// }

// py::list py_smc_pdo_buffer_stop_multi(WORD ConnectNo, WORD AxisNum, WORD* AxisList, WORD* ResultList) {
//     py::list ret;
//     ret.append(smc_pdo_buffer_stop_multi(ConnectNo, AxisNum, AxisList, ResultList));
//     return ret;
// }

// py::list py_nmcs_start_pdo_trace(WORD ConnectNo, WORD Channel, WORD SlaveAddr, WORD Index_Num, DWORD Trace_Len, WORD* Index, WORD* Sub_Index) {
//     py::list ret;
//     ret.append(nmcs_start_pdo_trace(ConnectNo, Channel, SlaveAddr, Index_Num, Trace_Len, Index, Sub_Index));
//     return ret;
// }

// py::list py_nmcs_get_pdo_trace(WORD ConnectNo, WORD Channel, WORD SlaveAddr, WORD* Index_Num, DWORD* Trace_Len, WORD* Index, WORD* Sub_Index) {
//     py::list ret;
//     ret.append(nmcs_get_pdo_trace(ConnectNo, Channel, SlaveAddr, Index_Num, Trace_Len, Index, Sub_Index));
//     return ret;
// }

// py::list py_nmcs_set_pdo_trace_trig_para(WORD ConnectNo, WORD Channel, WORD SlaveAddr, WORD Trig_Index, WORD Trig_Sub_Index, int Trig_Value, WORD Trig_Mode) {
//     py::list ret;
//     ret.append(nmcs_set_pdo_trace_trig_para(ConnectNo, Channel, SlaveAddr, Trig_Index, Trig_Sub_Index, Trig_Value, Trig_Mode));
//     return ret;
// }

// py::list py_nmcs_get_pdo_trace_trig_para(WORD ConnectNo, WORD Channel, WORD SlaveAddr, WORD* Trig_Index, WORD* Trig_Sub_Index, int* Trig_Value, WORD* Trig_Mode) {
//     py::list ret;
//     ret.append(nmcs_get_pdo_trace_trig_para(ConnectNo, Channel, SlaveAddr, Trig_Index, Trig_Sub_Index, Trig_Value, Trig_Mode));
//     return ret;
// }

// py::list py_nmcs_clear_pdo_trace_data(WORD ConnectNo, WORD Channel, WORD SlaveAddr) {
//     py::list ret;
//     ret.append(nmcs_clear_pdo_trace_data(ConnectNo, Channel, SlaveAddr));
//     return ret;
// }

// py::list py_nmcs_stop_pdo_trace(WORD ConnectNo, WORD Channel, WORD SlaveAddr) {
//     py::list ret;
//     ret.append(nmcs_stop_pdo_trace(ConnectNo, Channel, SlaveAddr));
//     return ret;
// }

// py::list py_nmcs_read_pdo_trace_data(WORD ConnectNo, WORD Channel, WORD SlaveAddr, DWORD StartAddr, DWORD Readlen, DWORD* ActReadlen, unsigned char* Data) {
//     py::list ret;
//     ret.append(nmcs_read_pdo_trace_data(ConnectNo, Channel, SlaveAddr, StartAddr, Readlen, ActReadlen, Data));
//     return ret;
// }

// py::list py_nmcs_get_pdo_trace_num(WORD ConnectNo, WORD Channel, WORD SlaveAddr, DWORD* Data_num, DWORD* Size_of_each_bag) {
//     py::list ret;
//     ret.append(nmcs_get_pdo_trace_num(ConnectNo, Channel, SlaveAddr, Data_num, Size_of_each_bag));
//     return ret;
// }

// py::list py_nmcs_get_pdo_trace_state(WORD ConnectNo, WORD Channel, WORD SlaveAddr, WORD* Trace_state) {
//     py::list ret;
//     ret.append(nmcs_get_pdo_trace_state(ConnectNo, Channel, SlaveAddr, Trace_state));
//     return ret;
// }

// py::list py_nmcs_get_current_fieldbus_state_info(WORD ConnectNo, WORD Channel, WORD* Axis, WORD* ErrorType, WORD* SlaveAddr, DWORD* ErrorFieldbusCode) {
//     py::list ret;
//     ret.append(nmcs_get_current_fieldbus_state_info(ConnectNo, Channel, Axis, ErrorType, SlaveAddr, ErrorFieldbusCode));
//     return ret;
// }

// py::list py_nmcs_get_detail_fieldbus_state_info(WORD ConnectNo, WORD Channel, DWORD ReadErrorNum, DWORD* TotalNum, DWORD* ActualNum, WORD* Axis, WORD* ErrorType, WORD* SlaveAddr, DWORD* ErrorFieldbusCode) {
//     py::list ret;
//     ret.append(nmcs_get_detail_fieldbus_state_info(ConnectNo, Channel, ReadErrorNum, TotalNum, ActualNum, Axis, ErrorType, SlaveAddr, ErrorFieldbusCode));
//     return ret;
// }

// py::list py_nmcs_reset_rtex(WORD ConnectNo) {
//     py::list ret;
//     ret.append(nmcs_reset_rtex(ConnectNo));
//     return ret;
// }

// py::list py_nmcs_start_connect(WORD ConnectNo, WORD chan, WORD* info, WORD* len) {
//     py::list ret;
//     ret.append(nmcs_start_connect(ConnectNo, chan, info, len));
//     return ret;
// }

// py::list py_nmcs_get_vendor_info(WORD ConnectNo, WORD axis, char* info, WORD* len) {
//     py::list ret;
//     ret.append(nmcs_get_vendor_info(ConnectNo, axis, info, len));
//     return ret;
// }

// py::list py_nmcs_get_slave_type_info(WORD ConnectNo, WORD axis, char* info, WORD* len) {
//     py::list ret;
//     ret.append(nmcs_get_slave_type_info(ConnectNo, axis, info, len));
//     return ret;
// }

// py::list py_nmcs_get_slave_name_info(WORD ConnectNo, WORD axis, char* info, WORD* len) {
//     py::list ret;
//     ret.append(nmcs_get_slave_name_info(ConnectNo, axis, info, len));
//     return ret;
// }

// py::list py_nmcs_get_slave_version_info(WORD ConnectNo, WORD axis, char* info, WORD* len) {
//     py::list ret;
//     ret.append(nmcs_get_slave_version_info(ConnectNo, axis, info, len));
//     return ret;
// }

// py::list py_nmcs_write_parameter(WORD ConnectNo, WORD axis, WORD index, WORD subindex, DWORD para_data) {
//     py::list ret;
//     ret.append(nmcs_write_parameter(ConnectNo, axis, index, subindex, para_data));
//     return ret;
// }

// py::list py_nmcs_write_slave_eeprom(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(nmcs_write_slave_eeprom(ConnectNo, axis));
//     return ret;
// }

// py::list py_nmcs_read_parameter(WORD ConnectNo, WORD axis, WORD index, WORD subindex, DWORD* para_data) {
//     py::list ret;
//     ret.append(nmcs_read_parameter(ConnectNo, axis, index, subindex, para_data));
//     return ret;
// }

// py::list py_nmcs_read_parameter_attributes(WORD ConnectNo, WORD axis, WORD index, WORD subindex, DWORD* para_data) {
//     py::list ret;
//     ret.append(nmcs_read_parameter_attributes(ConnectNo, axis, index, subindex, para_data));
//     return ret;
// }

// py::list py_nmcs_set_cmdcycletime(WORD ConnectNo, WORD PortNum, DWORD cmdtime) {
//     py::list ret;
//     ret.append(nmcs_set_cmdcycletime(ConnectNo, PortNum, cmdtime));
//     return ret;
// }

// py::list py_nmcs_get_cmdcycletime(WORD ConnectNo, WORD PortNum, DWORD* cmdtime) {
//     py::list ret;
//     ret.append(nmcs_get_cmdcycletime(ConnectNo, PortNum, cmdtime));
//     return ret;
// }

// py::list py_nmcs_start_log(WORD ConnectNo, WORD chan, WORD node, WORD Ifenable) {
//     py::list ret;
//     ret.append(nmcs_start_log(ConnectNo, chan, node, Ifenable));
//     return ret;
// }

// py::list py_nmcs_get_log(WORD ConnectNo, WORD chan, WORD node, DWORD* data) {
//     py::list ret;
//     ret.append(nmcs_get_log(ConnectNo, chan, node, data));
//     return ret;
// }

// py::list py_nmcs_config_atuo_log(WORD ConnectNo, WORD ifenable, WORD dir, WORD byte_index, WORD mask, WORD condition, DWORD counter) {
//     py::list ret;
//     ret.append(nmcs_config_atuo_log(ConnectNo, ifenable, dir, byte_index, mask, condition, counter));
//     return ret;
// }

// py::list py_nmcs_get_log_state(WORD ConnectNo, WORD chan, DWORD* state) {
//     py::list ret;
//     ret.append(nmcs_get_log_state(ConnectNo, chan, state));
//     return ret;
// }

// py::list py_nmcs_driver_reset(WORD ConnectNo, WORD axis) {
//     py::list ret;
//     ret.append(nmcs_driver_reset(ConnectNo, axis));
//     return ret;
// }

PYBIND11_MODULE(pyltsmc, m) {
    m.doc() = R"pbdoc(Python binding for leadshine smc motion controller by Pybind11.)pbdoc";

    m.def("smc_set_connect_timeout", &py_smc_set_connect_timeout, py::arg("timems"));
    m.def("smc_get_connect_status", &py_smc_get_connect_status, py::arg("ConnectNo"));
    m.def("smc_set_send_recv_timeout", &py_smc_set_send_recv_timeout, py::arg("SendTimems"), py::arg("RecvTimems"));
    m.def("smc_board_init", &py_smc_board_init, py::arg("ConnectNo"), py::arg("type"), py::arg("pconnectstring"), py::arg("dwBaudRate"));
    m.def("smc_board_init_ex", &py_smc_board_init_ex, py::arg("ConnectNo"), py::arg("type"), py::arg("pconnectstring"), py::arg("dwBaudRate"), py::arg("dwByteSize"), py::arg("dwParity"), py::arg("dwStopBits"));
    m.def("smc_board_close", &py_smc_board_close, py::arg("ConnectNo"));
    m.def("smc_soft_reset", &py_smc_soft_reset, py::arg("ConnectNo"));
    m.def("smc_board_reset", &py_smc_board_reset, py::arg("ConnectNo"));
    m.def("smc_set_debug_mode", &py_smc_set_debug_mode, py::arg("mode"), py::arg("FileName"));
    m.def("smc_get_debug_mode", &py_smc_get_debug_mode);
    m.def("smc_set_connect_debug_time", &py_smc_set_connect_debug_time, py::arg("ConnectNo"), py::arg("time_s"));
    m.def("smc_get_card_version", &py_smc_get_card_version, py::arg("ConnectNo"));
    m.def("smc_get_card_soft_version", &py_smc_get_card_soft_version, py::arg("ConnectNo"));
    m.def("smc_get_card_lib_version", &py_smc_get_card_lib_version);
    m.def("smc_get_release_version", &py_smc_get_release_version, py::arg("ConnectNo"));
    m.def("smc_get_total_axes", &py_smc_get_total_axes, py::arg("ConnectNo"));
    m.def("smc_get_total_ionum", &py_smc_get_total_ionum, py::arg("ConnectNo"));
    m.def("smc_get_total_adcnum", &py_smc_get_total_adcnum, py::arg("ConnectNo"));
    m.def("smc_format_flash", &py_smc_format_flash, py::arg("ConnectNo"));
    m.def("smc_rtc_get_time", &py_smc_rtc_get_time, py::arg("ConnectNo"));
    m.def("smc_rtc_set_time", &py_smc_rtc_set_time, py::arg("ConnectNo"), py::arg("year"), py::arg("month"), py::arg("day"), py::arg("hour"), py::arg("min"), py::arg("sec"));
    m.def("smc_set_ipaddr", &py_smc_set_ipaddr, py::arg("ConnectNo"), py::arg("IpAddr"));
    m.def("smc_get_ipaddr", &py_smc_get_ipaddr, py::arg("ConnectNo"));
    m.def("smc_set_com", &py_smc_set_com, py::arg("ConnectNo"), py::arg("com"), py::arg("dwBaudRate"), py::arg("wByteSize"), py::arg("wParity"), py::arg("wStopBits"));
    m.def("smc_get_com", &py_smc_get_com, py::arg("ConnectNo"), py::arg("com"));
    m.def("smc_write_sn", &py_smc_write_sn, py::arg("ConnectNo"), py::arg("sn"));
    m.def("smc_read_sn", &py_smc_read_sn, py::arg("ConnectNo"));
    m.def("smc_write_sn_numstring", &py_smc_write_sn_numstring, py::arg("ConnectNo"), py::arg("sn_str"));
    m.def("smc_read_sn_numstring", &py_smc_read_sn_numstring, py::arg("ConnectNo"));
    m.def("smc_write_password", &py_smc_write_password, py::arg("ConnectNo"), py::arg("str_pass"));
    m.def("smc_check_password", &py_smc_check_password, py::arg("ConnectNo"), py::arg("str_pass"));
    m.def("smc_enter_password", &py_smc_enter_password, py::arg("ConnectNo"), py::arg("str_pass"));
    m.def("smc_modify_password", &py_smc_modify_password, py::arg("ConnectNo"), py::arg("spassold"), py::arg("spass"));
    m.def("smc_download_parafile", &py_smc_download_parafile, py::arg("ConnectNo"), py::arg("FileName"));
    m.def("smc_upload_parafile", &py_smc_upload_parafile, py::arg("ConnectNo"), py::arg("FileName"));
    m.def("smc_set_el_mode", &py_smc_set_el_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("el_logic"), py::arg("el_mode"));
    m.def("smc_get_el_mode", &py_smc_get_el_mode, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_emg_mode", &py_smc_set_emg_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("emg_logic"));
    m.def("smc_get_emg_mode", &py_smc_get_emg_mode, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_softlimit_unit", &py_smc_set_softlimit_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("source_sel"), py::arg("SL_action"), py::arg("N_limit"), py::arg("P_limit"));
    m.def("smc_get_softlimit_unit", &py_smc_get_softlimit_unit, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_pulse_outmode", &py_smc_set_pulse_outmode, py::arg("ConnectNo"), py::arg("axis"), py::arg("outmode"));
    m.def("smc_get_pulse_outmode", &py_smc_get_pulse_outmode, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_equiv", &py_smc_set_equiv, py::arg("ConnectNo"), py::arg("axis"), py::arg("equiv"));
    m.def("smc_get_equiv", &py_smc_get_equiv, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_backlash_unit", &py_smc_set_backlash_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("backlash"));
    m.def("smc_get_backlash_unit", &py_smc_get_backlash_unit, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_axis_io_map", &py_smc_set_axis_io_map, py::arg("ConnectNo"), py::arg("Axis"), py::arg("IoType"), py::arg("MapIoType"), py::arg("MapIoIndex"), py::arg("Filter"));
    m.def("smc_get_axis_io_map", &py_smc_get_axis_io_map, py::arg("ConnectNo"), py::arg("Axis"), py::arg("IoType"));
    m.def("smc_set_profile_unit", &py_smc_set_profile_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("Min_Vel"), py::arg("Max_Vel"), py::arg("Tacc"), py::arg("Tdec"), py::arg("Stop_Vel"));
    m.def("smc_get_profile_unit", &py_smc_get_profile_unit, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_profile_unit_acc", &py_smc_set_profile_unit_acc, py::arg("ConnectNo"), py::arg("axis"), py::arg("Min_Vel"), py::arg("Max_Vel"), py::arg("acc"), py::arg("dec"), py::arg("Stop_Vel"));
    m.def("smc_get_profile_unit_acc", &py_smc_get_profile_unit_acc, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_s_profile", &py_smc_set_s_profile, py::arg("ConnectNo"), py::arg("axis"), py::arg("s_mode"), py::arg("s_para"));
    m.def("smc_get_s_profile", &py_smc_get_s_profile, py::arg("ConnectNo"), py::arg("axis"), py::arg("s_mode"));
    m.def("smc_set_dec_stop_time", &py_smc_set_dec_stop_time, py::arg("ConnectNo"), py::arg("axis"), py::arg("time"));
    m.def("smc_get_dec_stop_time", &py_smc_get_dec_stop_time, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_pmove_unit", &py_smc_pmove_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("Dist"), py::arg("posi_mode"));
    m.def("smc_vmove", &py_smc_vmove, py::arg("ConnectNo"), py::arg("axis"), py::arg("dir"));
    m.def("smc_change_speed_unit", &py_smc_change_speed_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("New_Vel"), py::arg("Taccdec"));
    m.def("smc_reset_target_position_unit", &py_smc_reset_target_position_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("New_Pos"));
    m.def("smc_update_target_position_unit", &py_smc_update_target_position_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("New_Pos"));
    m.def("smc_pmove_unit_extern", &py_smc_pmove_unit_extern, py::arg("ConnectNo"), py::arg("axis"), py::arg("MidPos"), py::arg("TargetPos"), py::arg("Min_Vel"), py::arg("Max_Vel"), py::arg("stop_Vel"), py::arg("acc"), py::arg("dec"), py::arg("smooth_time"), py::arg("posi_mode"));
    m.def("smc_set_plan_mode", &py_smc_set_plan_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("mode"));
    m.def("smc_get_plan_mode", &py_smc_get_plan_mode, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_pmove_sin_unit", &py_smc_pmove_sin_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("Dist"), py::arg("posi_mode"), py::arg("MaxVel"), py::arg("MaxAcc"));
    m.def("smc_pmove_change_pos_speed_config", &py_smc_pmove_change_pos_speed_config, py::arg("ConnectNo"), py::arg("axis"), py::arg("tar_vel"), py::arg("tar_rel_pos"), py::arg("trig_mode"), py::arg("source"));
    m.def("smc_get_pmove_change_pos_speed_config", &py_smc_get_pmove_change_pos_speed_config, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_pmove_change_pos_speed_enable", &py_smc_pmove_change_pos_speed_enable, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"));
    m.def("smc_get_pmove_change_pos_speed_enable", &py_smc_get_pmove_change_pos_speed_enable, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_get_pmove_change_pos_speed_state", &py_smc_get_pmove_change_pos_speed_state, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_home_pin_logic", &py_smc_set_home_pin_logic, py::arg("ConnectNo"), py::arg("axis"), py::arg("org_logic"), py::arg("filter"));
    m.def("smc_get_home_pin_logic", &py_smc_get_home_pin_logic, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_ez_mode", &py_smc_set_ez_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("ez_logic"), py::arg("ez_mode"), py::arg("filter"));
    m.def("smc_get_ez_mode", &py_smc_get_ez_mode, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_homemode", &py_smc_set_homemode, py::arg("ConnectNo"), py::arg("axis"), py::arg("home_dir"), py::arg("vel_mode"), py::arg("mode"), py::arg("pos_source"));
    m.def("smc_get_homemode", &py_smc_get_homemode, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_homespeed_unit", &py_smc_set_homespeed_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("homespeed"));
    m.def("smc_get_homespeed_unit", &py_smc_get_homespeed_unit, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_home_profile_unit", &py_smc_set_home_profile_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("Low_Vel"), py::arg("High_Vel"), py::arg("Tacc"), py::arg("Tdec"));
    m.def("smc_get_home_profile_unit", &py_smc_get_home_profile_unit, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_el_home", &py_smc_set_el_home, py::arg("ConnectNo"), py::arg("axis"), py::arg("mode"));
    m.def("smc_set_home_position_unit", &py_smc_set_home_position_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("position"));
    m.def("smc_get_home_position_unit", &py_smc_get_home_position_unit, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_home_move", &py_smc_home_move, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_get_home_result", &py_smc_get_home_result, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_pvt_table_unit", &py_smc_pvt_table_unit, py::arg("ConnectNo"), py::arg("iaxis"), py::arg("count"), py::arg("pTime"), py::arg("pPos"), py::arg("pVel"));
    // m.def("smc_pts_table_unit", &py_smc_pts_table_unit, py::arg("ConnectNo"), py::arg("iaxis"), py::arg("count"), py::arg("pTime"), py::arg("pPos"), py::arg("pPercent"));
    // m.def("smc_pvts_table_unit", &py_smc_pvts_table_unit, py::arg("ConnectNo"), py::arg("iaxis"), py::arg("count"), py::arg("pTime"), py::arg("pPos"), py::arg("velBegin"), py::arg("velEnd"));
    // m.def("smc_ptt_table_unit", &py_smc_ptt_table_unit, py::arg("ConnectNo"), py::arg("iaxis"), py::arg("count"), py::arg("pTime"), py::arg("pPos"));
    // m.def("smc_pvt_move", &py_smc_pvt_move, py::arg("ConnectNo"), py::arg("AxisNum"), py::arg("AxisList"));
    // m.def("smc_cam_table_unit", &py_smc_cam_table_unit, py::arg("ConnectNo"), py::arg("MasterAxisNo"), py::arg("SlaveAxisNo"), py::arg("Count"), py::arg("pMasterPos"), py::arg("pSlavePos"), py::arg("SrcMode"));
    m.def("smc_cam_move", &py_smc_cam_move, py::arg("ConnectNo"), py::arg("AxisNo"));
    m.def("smc_sine_oscillate_unit", &py_smc_sine_oscillate_unit, py::arg("ConnectNo"), py::arg("Axis"), py::arg("Amplitude"), py::arg("Frequency"));
    m.def("smc_sine_oscillate_stop", &py_smc_sine_oscillate_stop, py::arg("ConnectNo"), py::arg("Axis"));
    m.def("smc_handwheel_set_axislist", &py_smc_handwheel_set_axislist, py::arg("ConnectNo"), py::arg("AxisSelIndex"), py::arg("AxisNum"), py::arg("AxisList"));
    m.def("smc_handwheel_get_axislist", &py_smc_handwheel_get_axislist, py::arg("ConnectNo"), py::arg("AxisSelIndex"));
    m.def("smc_handwheel_set_ratiolist", &py_smc_handwheel_set_ratiolist, py::arg("ConnectNo"), py::arg("AxisSelIndex"), py::arg("StartRatioIndex"), py::arg("RatioSelNum"), py::arg("RatioList"));
    m.def("smc_handwheel_get_ratiolist", &py_smc_handwheel_get_ratiolist, py::arg("ConnectNo"), py::arg("AxisSelIndex"), py::arg("StartRatioIndex"), py::arg("RatioSelNum"));
    m.def("smc_handwheel_set_mode", &py_smc_handwheel_set_mode, py::arg("ConnectNo"), py::arg("InMode"), py::arg("IfHardEnable"));
    m.def("smc_handwheel_get_mode", &py_smc_handwheel_get_mode, py::arg("ConnectNo"));
    m.def("smc_handwheel_set_index", &py_smc_handwheel_set_index, py::arg("ConnectNo"), py::arg("AxisSelIndex"), py::arg("RatioSelIndex"));
    m.def("smc_handwheel_get_index", &py_smc_handwheel_get_index, py::arg("ConnectNo"));
    m.def("smc_handwheel_move", &py_smc_handwheel_move, py::arg("ConnectNo"), py::arg("ForceMove"));
    m.def("smc_handwheel_stop", &py_smc_handwheel_stop, py::arg("ConnectNo"));
    m.def("smc_set_handwheel_inmode", &py_smc_set_handwheel_inmode, py::arg("ConnectNo"), py::arg("axis"), py::arg("inmode"), py::arg("multi"), py::arg("vh"));
    m.def("smc_get_handwheel_inmode", &py_smc_get_handwheel_inmode, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_handwheel_inmode_extern", &py_smc_set_handwheel_inmode_extern, py::arg("ConnectNo"), py::arg("inmode"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("multi"));
    m.def("smc_get_handwheel_inmode_extern", &py_smc_get_handwheel_inmode_extern, py::arg("ConnectNo"));
    m.def("smc_set_handwheel_inmode_decimals", &py_smc_set_handwheel_inmode_decimals, py::arg("ConnectNo"), py::arg("axis"), py::arg("inmode"), py::arg("multi"), py::arg("vh"));
    m.def("smc_get_handwheel_inmode_decimals", &py_smc_get_handwheel_inmode_decimals, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_handwheel_inmode_extern_decimals", &py_smc_set_handwheel_inmode_extern_decimals, py::arg("ConnectNo"), py::arg("inmode"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("multi"));
    m.def("smc_get_handwheel_inmode_extern_decimals", &py_smc_get_handwheel_inmode_extern_decimals, py::arg("ConnectNo"));
    m.def("smc_set_vector_profile_unit", &py_smc_set_vector_profile_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("Min_Vel"), py::arg("Max_Vel"), py::arg("Tacc"), py::arg("Tdec"), py::arg("Stop_Vel"));
    m.def("smc_get_vector_profile_unit", &py_smc_get_vector_profile_unit, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_set_vector_profile_unit_acc", &py_smc_set_vector_profile_unit_acc, py::arg("ConnectNo"), py::arg("Crd"), py::arg("Min_Vel"), py::arg("Max_Vel"), py::arg("acc"), py::arg("dec"), py::arg("Stop_Vel"));
    m.def("smc_get_vector_profile_unit_acc", &py_smc_get_vector_profile_unit_acc, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_set_vector_s_profile", &py_smc_set_vector_s_profile, py::arg("ConnectNo"), py::arg("Crd"), py::arg("s_mode"), py::arg("s_para"));
    m.def("smc_get_vector_s_profile", &py_smc_get_vector_s_profile, py::arg("ConnectNo"), py::arg("Crd"), py::arg("s_mode"));
    m.def("smc_set_vector_dec_stop_time", &py_smc_set_vector_dec_stop_time, py::arg("ConnectNo"), py::arg("Crd"), py::arg("time"));
    m.def("smc_get_vector_dec_stop_time", &py_smc_get_vector_dec_stop_time, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_line_unit", &py_smc_line_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("Dist"), py::arg("posi_mode"));
    m.def("smc_arc_move_center_unit", &py_smc_arc_move_center_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("Target_Pos"), py::arg("Cen_Pos"), py::arg("Arc_Dir"), py::arg("Circle"), py::arg("posi_mode"));
    m.def("smc_arc_move_radius_unit", &py_smc_arc_move_radius_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("Target_Pos"), py::arg("Arc_Radius"), py::arg("Arc_Dir"), py::arg("Circle"), py::arg("posi_mode"));
    m.def("smc_arc_move_3points_unit", &py_smc_arc_move_3points_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("Target_Pos"), py::arg("Mid_Pos"), py::arg("Circle"), py::arg("posi_mode"));
    m.def("smc_conti_set_lookahead_mode", &py_smc_conti_set_lookahead_mode, py::arg("ConnectNo"), py::arg("Crd"), py::arg("enable"), py::arg("LookaheadSegments"), py::arg("PathError"), py::arg("LookaheadAcc"));
    m.def("smc_conti_get_lookahead_mode", &py_smc_conti_get_lookahead_mode, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_set_arc_limit", &py_smc_set_arc_limit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("Enable"), py::arg("0"), py::arg("0"));
    m.def("smc_get_arc_limit", &py_smc_get_arc_limit, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_conti_open_list", &py_smc_conti_open_list, py::arg("ConnectNo"), py::arg("Crd"), py::arg("AxisNum"), py::arg("AxisList"));
    m.def("smc_conti_close_list", &py_smc_conti_close_list, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_conti_stop_list", &py_smc_conti_stop_list, py::arg("ConnectNo"), py::arg("Crd"), py::arg("stop_mode"));
    m.def("smc_conti_pause_list", &py_smc_conti_pause_list, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_conti_start_list", &py_smc_conti_start_list, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_conti_change_speed_ratio", &py_smc_conti_change_speed_ratio, py::arg("ConnectNo"), py::arg("Crd"), py::arg("percent"));
    m.def("smc_conti_get_run_state", &py_smc_conti_get_run_state, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_conti_remain_space", &py_smc_conti_remain_space, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_conti_read_current_mark", &py_smc_conti_read_current_mark, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_conti_line_unit", &py_smc_conti_line_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("pPosList"), py::arg("posi_mode"), py::arg("mark"));
    m.def("smc_conti_arc_move_center_unit", &py_smc_conti_arc_move_center_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("Target_Pos"), py::arg("Cen_Pos"), py::arg("Arc_Dir"), py::arg("Circle"), py::arg("posi_mode"), py::arg("mark"));
    m.def("smc_conti_arc_move_radius_unit", &py_smc_conti_arc_move_radius_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("Target_Pos"), py::arg("Arc_Radius"), py::arg("Arc_Dir"), py::arg("Circle"), py::arg("posi_mode"), py::arg("mark"));
    m.def("smc_conti_arc_move_3points_unit", &py_smc_conti_arc_move_3points_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("Target_Pos"), py::arg("Mid_Pos"), py::arg("Circle"), py::arg("posi_mode"), py::arg("mark"));
    m.def("smc_conti_wait_input", &py_smc_conti_wait_input, py::arg("ConnectNo"), py::arg("Crd"), py::arg("bitno"), py::arg("on_off"), py::arg("TimeOut"), py::arg("mark"));
    m.def("smc_conti_delay_outbit_to_start", &py_smc_conti_delay_outbit_to_start, py::arg("ConnectNo"), py::arg("Crd"), py::arg("bitno"), py::arg("on_off"), py::arg("delay_value"), py::arg("delay_mode"), py::arg("ReverseTime"));
    m.def("smc_conti_delay_outbit_to_stop", &py_smc_conti_delay_outbit_to_stop, py::arg("ConnectNo"), py::arg("Crd"), py::arg("bitno"), py::arg("on_off"), py::arg("delay_time"), py::arg("ReverseTime"));
    m.def("smc_conti_ahead_outbit_to_stop", &py_smc_conti_ahead_outbit_to_stop, py::arg("ConnectNo"), py::arg("Crd"), py::arg("bitno"), py::arg("on_off"), py::arg("ahead_value"), py::arg("ahead_mode"), py::arg("ReverseTime"));
    m.def("smc_conti_accurate_outbit_unit", &py_smc_conti_accurate_outbit_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("cmp_no"), py::arg("on_off"), py::arg("axis"), py::arg("abs_pos"), py::arg("pos_source"), py::arg("ReverseTime"));
    m.def("smc_conti_write_outbit", &py_smc_conti_write_outbit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("bitno"), py::arg("on_off"), py::arg("ReverseTime"));
    m.def("smc_conti_clear_io_action", &py_smc_conti_clear_io_action, py::arg("ConnectNo"), py::arg("Crd"), py::arg("Io_Mask"));
    m.def("smc_conti_set_pause_output", &py_smc_conti_set_pause_output, py::arg("ConnectNo"), py::arg("Crd"), py::arg("action"), py::arg("mask"), py::arg("state"));
    m.def("smc_conti_get_pause_output", &py_smc_conti_get_pause_output, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_conti_set_override", &py_smc_conti_set_override, py::arg("ConnectNo"), py::arg("Crd"), py::arg("Percent"));
    m.def("smc_conti_set_blend", &py_smc_conti_set_blend, py::arg("ConnectNo"), py::arg("Crd"), py::arg("enable"));
    m.def("smc_conti_get_blend", &py_smc_conti_get_blend, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_conti_pmove_unit", &py_smc_conti_pmove_unit, py::arg("ConnectNo"), py::arg("Crd"), py::arg("axis"), py::arg("dist"), py::arg("posi_mode"), py::arg("mode"), py::arg("mark"));
    m.def("smc_conti_delay", &py_smc_conti_delay, py::arg("ConnectNo"), py::arg("Crd"), py::arg("delay_time"), py::arg("mark"));
    m.def("smc_set_pwm_enable", &py_smc_set_pwm_enable, py::arg("ConnectNo"), py::arg("pwmno"), py::arg("enable"));
    m.def("smc_get_pwm_enable", &py_smc_get_pwm_enable, py::arg("ConnectNo"), py::arg("pwmno"));
    m.def("smc_set_pwm_output", &py_smc_set_pwm_output, py::arg("ConnectNo"), py::arg("pwmno"), py::arg("fDuty"), py::arg("fFre"));
    m.def("smc_get_pwm_output", &py_smc_get_pwm_output, py::arg("ConnectNo"), py::arg("pwmno"));
    m.def("smc_conti_set_pwm_output", &py_smc_conti_set_pwm_output, py::arg("ConnectNo"), py::arg("Crd"), py::arg("pwmno"), py::arg("fDuty"), py::arg("fFre"));
    m.def("smc_set_pwm_follow_speed", &py_smc_set_pwm_follow_speed, py::arg("ConnectNo"), py::arg("pwmno"), py::arg("mode"), py::arg("MaxVel"), py::arg("MaxValue"), py::arg("OutValue"));
    m.def("smc_get_pwm_follow_speed", &py_smc_get_pwm_follow_speed, py::arg("ConnectNo"), py::arg("pwmno"));
    m.def("smc_set_pwm_onoff_duty", &py_smc_set_pwm_onoff_duty, py::arg("ConnectNo"), py::arg("pwmno"), py::arg("fOnDuty"), py::arg("fOffDuty"));
    m.def("smc_get_pwm_onoff_duty", &py_smc_get_pwm_onoff_duty, py::arg("ConnectNo"), py::arg("pwmno"));
    m.def("smc_set_pwm_follow_onoff", &py_smc_set_pwm_follow_onoff, py::arg("ConnectNo"), py::arg("pwmno"), py::arg("Crd"), py::arg("on_off"));
    m.def("smc_get_pwm_follow_onoff", &py_smc_get_pwm_follow_onoff, py::arg("ConnectNo"), py::arg("pwmno"));
    m.def("smc_conti_delay_pwm_to_start", &py_smc_conti_delay_pwm_to_start, py::arg("ConnectNo"), py::arg("Crd"), py::arg("pwmno"), py::arg("on_off"), py::arg("delay_value"), py::arg("delay_mode"), py::arg("ReverseTime"));
    m.def("smc_conti_ahead_pwm_to_stop", &py_smc_conti_ahead_pwm_to_stop, py::arg("ConnectNo"), py::arg("Crd"), py::arg("pwmno"), py::arg("on_off"), py::arg("ahead_value"), py::arg("ahead_mode"), py::arg("ReverseTime"));
    m.def("smc_conti_write_pwm", &py_smc_conti_write_pwm, py::arg("ConnectNo"), py::arg("Crd"), py::arg("pwmno"), py::arg("on_off"), py::arg("ReverseTime"));
    m.def("smc_laser_set_output", &py_smc_laser_set_output, py::arg("ConnectNo"), py::arg("Enable"), py::arg("Width"));
    m.def("smc_laser_get_output", &py_smc_laser_get_output, py::arg("ConnectNo"));
    m.def("smc_set_counter_inmode", &py_smc_set_counter_inmode, py::arg("ConnectNo"), py::arg("axis"), py::arg("mode"));
    m.def("smc_get_counter_inmode", &py_smc_get_counter_inmode, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_counter_reverse", &py_smc_set_counter_reverse, py::arg("ConnectNo"), py::arg("axis"), py::arg("reverse"));
    m.def("smc_get_counter_reverse", &py_smc_get_counter_reverse, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_encoder_unit", &py_smc_set_encoder_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"));
    m.def("smc_get_encoder_unit", &py_smc_get_encoder_unit, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_set_extra_encoder_mode", &py_smc_set_extra_encoder_mode, py::arg("ConnectNo"), py::arg("channel"), py::arg("inmode"), py::arg("multi"));
    m.def("smc_get_extra_encoder_mode", &py_smc_get_extra_encoder_mode, py::arg("ConnectNo"), py::arg("channel"));
    m.def("smc_set_extra_encoder", &py_smc_set_extra_encoder, py::arg("ConnectNo"), py::arg("channel"), py::arg("pos"));
    m.def("smc_get_extra_encoder", &py_smc_get_extra_encoder, py::arg("ConnectNo"), py::arg("channel"));
    m.def("smc_read_inbit", &py_smc_read_inbit, py::arg("ConnectNo"), py::arg("bitno"));
    m.def("smc_write_outbit", &py_smc_write_outbit, py::arg("ConnectNo"), py::arg("bitno"), py::arg("on_off"));
    m.def("smc_read_outbit", &py_smc_read_outbit, py::arg("ConnectNo"), py::arg("bitno"));
    m.def("smc_read_inport", &py_smc_read_inport, py::arg("ConnectNo"), py::arg("portno"));
    m.def("smc_read_outport", &py_smc_read_outport, py::arg("ConnectNo"), py::arg("portno"));
    m.def("smc_write_outport", &py_smc_write_outport, py::arg("ConnectNo"), py::arg("portno"), py::arg("outport_val"));
    m.def("smc_read_inbit_ex", &py_smc_read_inbit_ex, py::arg("ConnectNo"), py::arg("bitno"));
    m.def("smc_read_outbit_ex", &py_smc_read_outbit_ex, py::arg("ConnectNo"), py::arg("bitno"));
    m.def("smc_read_inport_ex", &py_smc_read_inport_ex, py::arg("ConnectNo"), py::arg("portno"));
    m.def("smc_read_outport_ex", &py_smc_read_outport_ex, py::arg("ConnectNo"), py::arg("portno"));
    m.def("smc_reverse_outbit", &py_smc_reverse_outbit, py::arg("ConnectNo"), py::arg("bitno"), py::arg("reverse_time"));
    m.def("smc_set_outbit_delay_reverse", &py_smc_set_outbit_delay_reverse, py::arg("ConnectNo"), py::arg("channel"), py::arg("outbit"), py::arg("outlevel"), py::arg("outtime"), py::arg("outmode"));
    m.def("smc_set_io_pwmoutput", &py_smc_set_io_pwmoutput, py::arg("ConnectNo"), py::arg("outbit"), py::arg("time1"), py::arg("time2"), py::arg("counts"));
    m.def("smc_clear_io_pwmoutput", &py_smc_clear_io_pwmoutput, py::arg("ConnectNo"), py::arg("outbit"));
    m.def("smc_set_io_count_mode", &py_smc_set_io_count_mode, py::arg("ConnectNo"), py::arg("bitno"), py::arg("mode"), py::arg("filter"));
    m.def("smc_get_io_count_mode", &py_smc_get_io_count_mode, py::arg("ConnectNo"), py::arg("bitno"));
    m.def("smc_set_io_count_value", &py_smc_set_io_count_value, py::arg("ConnectNo"), py::arg("bitno"), py::arg("CountValue"));
    m.def("smc_get_io_count_value", &py_smc_get_io_count_value, py::arg("ConnectNo"), py::arg("bitno"));
    m.def("smc_set_io_map_virtual", &py_smc_set_io_map_virtual, py::arg("ConnectNo"), py::arg("bitno"), py::arg("MapIoType"), py::arg("MapIoIndex"), py::arg("Filter"));
    m.def("smc_get_io_map_virtual", &py_smc_get_io_map_virtual, py::arg("ConnectNo"), py::arg("bitno"));
    m.def("smc_read_inbit_virtual", &py_smc_read_inbit_virtual, py::arg("ConnectNo"), py::arg("bitno"));
    // m.def("smc_set_io_dstp_mode", &py_smc_set_io_dstp_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("logic"));
    // m.def("smc_get_io_dstp_mode", &py_smc_get_io_dstp_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("logic"));
    // m.def("smc_set_alm_mode", &py_smc_set_alm_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("alm_logic"), py::arg("alm_action"));
    // m.def("smc_get_alm_mode", &py_smc_get_alm_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("alm_logic"), py::arg("alm_action"));
    // m.def("smc_set_inp_mode", &py_smc_set_inp_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("inp_logic"));
    // m.def("smc_get_inp_mode", &py_smc_get_inp_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("inp_logic"));
    // m.def("smc_write_sevon_pin", &py_smc_write_sevon_pin, py::arg("ConnectNo"), py::arg("axis"), py::arg("on_off"));
    // m.def("smc_read_sevon_pin", &py_smc_read_sevon_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_write_erc_pin", &py_smc_write_erc_pin, py::arg("ConnectNo"), py::arg("axis"), py::arg("on_off"));
    // m.def("smc_read_erc_pin", &py_smc_read_erc_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_read_alarm_pin", &py_smc_read_alarm_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_read_inp_pin", &py_smc_read_inp_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_read_org_pin", &py_smc_read_org_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_read_elp_pin", &py_smc_read_elp_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_read_eln_pin", &py_smc_read_eln_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_read_emg_pin", &py_smc_read_emg_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_read_ez_pin", &py_smc_read_ez_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_read_rdy_pin", &py_smc_read_rdy_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_read_cmp_pin", &py_smc_read_cmp_pin, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_write_cmp_pin", &py_smc_write_cmp_pin, py::arg("ConnectNo"), py::arg("axis"), py::arg("on_off"));
    // m.def("smc_read_sevon_pin_ex", &py_smc_read_sevon_pin_ex, py::arg("ConnectNo"), py::arg("axis"), py::arg("state"));
    // m.def("smc_read_erc_pin_ex", &py_smc_read_erc_pin_ex, py::arg("ConnectNo"), py::arg("axis"), py::arg("state"));
    // m.def("smc_read_alarm_pin_ex", &py_smc_read_alarm_pin_ex, py::arg("ConnectNo"), py::arg("axis"), py::arg("state"));
    // m.def("smc_read_inp_pin_ex", &py_smc_read_inp_pin_ex, py::arg("ConnectNo"), py::arg("axis"), py::arg("state"));
    // m.def("smc_read_org_pin_ex", &py_smc_read_org_pin_ex, py::arg("ConnectNo"), py::arg("uiaxis"), py::arg("state"));
    // m.def("smc_read_elp_pin_ex", &py_smc_read_elp_pin_ex, py::arg("ConnectNo"), py::arg("uiaxis"), py::arg("state"));
    // m.def("smc_read_eln_pin_ex", &py_smc_read_eln_pin_ex, py::arg("ConnectNo"), py::arg("uiaxis"), py::arg("state"));
    // m.def("smc_read_emg_pin_ex", &py_smc_read_emg_pin_ex, py::arg("ConnectNo"), py::arg("uiaxis"), py::arg("state"));
    // m.def("smc_read_ez_pin_ex", &py_smc_read_ez_pin_ex, py::arg("ConnectNo"), py::arg("axis"), py::arg("state"));
    // m.def("smc_read_rdy_pin_ex", &py_smc_read_rdy_pin_ex, py::arg("ConnectNo"), py::arg("axis"), py::arg("state"));
    // m.def("smc_compare_set_config", &py_smc_compare_set_config, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("cmp_source"));
    // m.def("smc_compare_get_config", &py_smc_compare_get_config, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("cmp_source"));
    // m.def("smc_compare_clear_points", &py_smc_compare_clear_points, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_compare_add_point_unit", &py_smc_compare_add_point_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"), py::arg("dir"), py::arg("action"), py::arg("actpara"));
    // m.def("smc_compare_add_point_cycle", &py_smc_compare_add_point_cycle, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"), py::arg("dir"), py::arg("bitno"), py::arg("cycle"), py::arg("level"));
    // m.def("smc_compare_get_current_point_unit", &py_smc_compare_get_current_point_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"));
    // m.def("smc_compare_get_points_runned", &py_smc_compare_get_points_runned, py::arg("ConnectNo"), py::arg("axis"), py::arg("pointNum"));
    // m.def("smc_compare_get_points_remained", &py_smc_compare_get_points_remained, py::arg("ConnectNo"), py::arg("axis"), py::arg("pointNum"));
    // m.def("smc_compare_set_config_extern", &py_smc_compare_set_config_extern, py::arg("ConnectNo"), py::arg("enable"), py::arg("cmp_source"));
    // m.def("smc_compare_get_config_extern", &py_smc_compare_get_config_extern, py::arg("ConnectNo"), py::arg("enable"), py::arg("cmp_source"));
    // m.def("smc_compare_clear_points_extern", &py_smc_compare_clear_points_extern, py::arg("ConnectNo"));
    // m.def("smc_compare_add_point_extern_unit", &py_smc_compare_add_point_extern_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"), py::arg("dir"), py::arg("action"), py::arg("actpara"));
    // m.def("smc_compare_add_point_cycle_2d", &py_smc_compare_add_point_cycle_2d, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"), py::arg("dir"), py::arg("bitno"), py::arg("cycle"), py::arg("level"));
    // m.def("smc_compare_get_current_point_extern_unit", &py_smc_compare_get_current_point_extern_unit, py::arg("ConnectNo"), py::arg("pos"));
    // m.def("smc_compare_get_points_runned_extern", &py_smc_compare_get_points_runned_extern, py::arg("ConnectNo"), py::arg("pointNum"));
    // m.def("smc_compare_get_points_remained_extern", &py_smc_compare_get_points_remained_extern, py::arg("ConnectNo"), py::arg("pointNum"));
    // m.def("smc_hcmp_set_mode", &py_smc_hcmp_set_mode, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("cmp_mode"));
    // m.def("smc_hcmp_get_mode", &py_smc_hcmp_get_mode, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("cmp_mode"));
    // m.def("smc_hcmp_set_config", &py_smc_hcmp_set_config, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("axis"), py::arg("cmp_source"), py::arg("cmp_logic"), py::arg("time"));
    // m.def("smc_hcmp_get_config", &py_smc_hcmp_get_config, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("axis"), py::arg("cmp_source"), py::arg("cmp_logic"), py::arg("time"));
    // m.def("smc_hcmp_add_point_unit", &py_smc_hcmp_add_point_unit, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("cmp_pos"));
    // m.def("smc_hcmp_set_liner_unit", &py_smc_hcmp_set_liner_unit, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("Increment"), py::arg("Count"));
    // m.def("smc_hcmp_get_liner_unit", &py_smc_hcmp_get_liner_unit, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("Increment"), py::arg("Count"));
    // m.def("smc_hcmp_get_current_state_unit", &py_smc_hcmp_get_current_state_unit, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("remained_points"), py::arg("current_point"), py::arg("runned_points"));
    // m.def("smc_hcmp_clear_points", &py_smc_hcmp_clear_points, py::arg("ConnectNo"), py::arg("hcmp"));
    // m.def("smc_hcmp_2d_set_enable", &py_smc_hcmp_2d_set_enable, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("cmp_enable"));
    // m.def("smc_hcmp_2d_get_enable", &py_smc_hcmp_2d_get_enable, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("cmp_enable"));
    // m.def("smc_hcmp_2d_set_config_unit", &py_smc_hcmp_2d_set_config_unit, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("cmp_mode"), py::arg("x_axis"), py::arg("x_cmp_source"), py::arg("x_cmp_error"), py::arg("y_axis"), py::arg("y_cmp_source"), py::arg("y_cmp_error"), py::arg("cmp_logic"), py::arg("time"));
    // m.def("smc_hcmp_2d_get_config_unit", &py_smc_hcmp_2d_get_config_unit, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("cmp_mode"), py::arg("x_axis"), py::arg("x_cmp_source"), py::arg("x_cmp_error"), py::arg("y_axis"), py::arg("y_cmp_source"), py::arg("y_cmp_error"), py::arg("cmp_logic"), py::arg("time"));
    // m.def("smc_hcmp_2d_set_pwmoutput", &py_smc_hcmp_2d_set_pwmoutput, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("pwm_enable"), py::arg("duty"), py::arg("freq"), py::arg("pwm_number"));
    // m.def("smc_hcmp_2d_get_pwmoutput", &py_smc_hcmp_2d_get_pwmoutput, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("pwm_enable"), py::arg("duty"), py::arg("freq"), py::arg("pwm_number"));
    // m.def("smc_hcmp_2d_add_point_unit", &py_smc_hcmp_2d_add_point_unit, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("x_cmp_pos"), py::arg("y_cmp_pos"), py::arg("cmp_outbit"));
    // m.def("smc_hcmp_2d_get_current_state_unit", &py_smc_hcmp_2d_get_current_state_unit, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("remained_points"), py::arg("x_current_point"), py::arg("y_current_point"), py::arg("runned_points"), py::arg("current_state"), py::arg("current_outbit"));
    // m.def("smc_hcmp_2d_clear_points", &py_smc_hcmp_2d_clear_points, py::arg("ConnectNo"), py::arg("hcmp"));
    // m.def("smc_hcmp_2d_force_output", &py_smc_hcmp_2d_force_output, py::arg("ConnectNo"), py::arg("hcmp"), py::arg("outbit"));
    // m.def("smc_set_homelatch_mode", &py_smc_set_homelatch_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("logic"), py::arg("source"));
    // m.def("smc_get_homelatch_mode", &py_smc_get_homelatch_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("logic"), py::arg("source"));
    // m.def("smc_get_homelatch_flag", &py_smc_get_homelatch_flag, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_reset_homelatch_flag", &py_smc_reset_homelatch_flag, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_get_homelatch_value_unit", &py_smc_get_homelatch_value_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos_by_mm"));
    // m.def("smc_set_ezlatch_mode", &py_smc_set_ezlatch_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("logic"), py::arg("source"));
    // m.def("smc_get_ezlatch_mode", &py_smc_get_ezlatch_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("enable"), py::arg("logic"), py::arg("source"));
    // m.def("smc_get_ezlatch_flag", &py_smc_get_ezlatch_flag, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_reset_ezlatch_flag", &py_smc_reset_ezlatch_flag, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_get_ezlatch_value_unit", &py_smc_get_ezlatch_value_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos_by_mm"));
    // m.def("smc_set_ltc_mode", &py_smc_set_ltc_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("ltc_logic"), py::arg("ltc_mode"), py::arg("filter"));
    // m.def("smc_get_ltc_mode", &py_smc_get_ltc_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("ltc_logic"), py::arg("ltc_mode"), py::arg("filter"));
    // m.def("smc_set_latch_mode", &py_smc_set_latch_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("all_enable"), py::arg("latch_source"), py::arg("triger_chunnel"));
    // m.def("smc_get_latch_mode", &py_smc_get_latch_mode, py::arg("ConnectNo"), py::arg("axis"), py::arg("all_enable"), py::arg("latch_source"), py::arg("triger_chunnel"));
    // m.def("smc_get_latch_flag", &py_smc_get_latch_flag, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_reset_latch_flag", &py_smc_reset_latch_flag, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_get_latch_value_unit", &py_smc_get_latch_value_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos_by_mm"));
    // m.def("smc_ltc_set_mode", &py_smc_ltc_set_mode, py::arg("ConnectNo"), py::arg("latch"), py::arg("ltc_mode"), py::arg("ltc_logic"), py::arg("filter"));
    // m.def("smc_ltc_get_mode", &py_smc_ltc_get_mode, py::arg("ConnectNo"), py::arg("latch"), py::arg("ltc_mode"), py::arg("ltc_logic"), py::arg("filter"));
    // m.def("smc_ltc_set_source", &py_smc_ltc_set_source, py::arg("ConnectNo"), py::arg("latch"), py::arg("axis"), py::arg("ltc_source"));
    // m.def("smc_ltc_get_source", &py_smc_ltc_get_source, py::arg("ConnectNo"), py::arg("latch"), py::arg("axis"), py::arg("ltc_source"));
    // m.def("smc_ltc_reset", &py_smc_ltc_reset, py::arg("ConnectNo"), py::arg("latch"));
    // m.def("smc_ltc_get_number", &py_smc_ltc_get_number, py::arg("ConnectNo"), py::arg("latch"), py::arg("axis"), py::arg("number"));
    // m.def("smc_ltc_get_value_unit", &py_smc_ltc_get_value_unit, py::arg("ConnectNo"), py::arg("latch"), py::arg("axis"), py::arg("value"));
    // m.def("smc_softltc_set_mode", &py_smc_softltc_set_mode, py::arg("ConnectNo"), py::arg("latch"), py::arg("ltc_enable"), py::arg("ltc_mode"), py::arg("ltc_inbit"), py::arg("ltc_logic"), py::arg("filter"));
    // m.def("smc_softltc_get_mode", &py_smc_softltc_get_mode, py::arg("ConnectNo"), py::arg("latch"), py::arg("ltc_enable"), py::arg("ltc_mode"), py::arg("ltc_inbit"), py::arg("ltc_logic"), py::arg("filter"));
    // m.def("smc_softltc_set_source", &py_smc_softltc_set_source, py::arg("ConnectNo"), py::arg("latch"), py::arg("axis"), py::arg("ltc_source"));
    // m.def("smc_softltc_get_source", &py_smc_softltc_get_source, py::arg("ConnectNo"), py::arg("latch"), py::arg("axis"), py::arg("ltc_source"));
    // m.def("smc_softltc_reset", &py_smc_softltc_reset, py::arg("ConnectNo"), py::arg("latch"));
    // m.def("smc_softltc_get_number", &py_smc_softltc_get_number, py::arg("ConnectNo"), py::arg("latch"), py::arg("axis"), py::arg("number"));
    // m.def("smc_softltc_get_value_unit", &py_smc_softltc_get_value_unit, py::arg("ConnectNo"), py::arg("latch"), py::arg("axis"), py::arg("value"));
    // m.def("smc_set_ain_action", &py_smc_set_ain_action, py::arg("ConnectNo"), py::arg("channel"), py::arg("mode"), py::arg("fvoltage"), py::arg("action"), py::arg("actpara"));
    // m.def("smc_get_ain_action", &py_smc_get_ain_action, py::arg("ConnectNo"), py::arg("channel"), py::arg("mode"), py::arg("fvoltage"), py::arg("action"), py::arg("actpara"));
    // m.def("smc_get_ain_state", &py_smc_get_ain_state, py::arg("ConnectNo"), py::arg("channel"));
    // m.def("smc_set_ain_state", &py_smc_set_ain_state, py::arg("ConnectNo"), py::arg("channel"));
    // m.def("smc_get_ain", &py_smc_get_ain, py::arg("ConnectNo"), py::arg("channel"));
    // m.def("smc_set_da_output", &py_smc_set_da_output, py::arg("ConnectNo"), py::arg("channel"), py::arg("Vout"));
    // m.def("smc_get_da_output", &py_smc_get_da_output, py::arg("ConnectNo"), py::arg("channel"), py::arg("Vout"));
    // m.def("smc_download_file", &py_smc_download_file, py::arg("ConnectNo"), py::arg("pfilename"), py::arg("pfilenameinControl"), py::arg("filetype"));
    // m.def("smc_download_memfile", &py_smc_download_memfile, py::arg("ConnectNo"), py::arg("pbuffer"), py::arg("buffsize"), py::arg("pfilenameinControl"), py::arg("filetype"));
    // m.def("smc_upload_file", &py_smc_upload_file, py::arg("ConnectNo"), py::arg("pfilename"), py::arg("pfilenameinControl"), py::arg("filetype"));
    // m.def("smc_upload_memfile", &py_smc_upload_memfile, py::arg("ConnectNo"), py::arg("pbuffer"), py::arg("buffsize"), py::arg("pfilenameinControl"), py::arg("puifilesize"), py::arg("filetype"));
    // m.def("smc_download_file_to_ram", &py_smc_download_file_to_ram, py::arg("ConnectNo"), py::arg("pfilename"), py::arg("filetype"));
    // m.def("smc_download_memfile_to_ram", &py_smc_download_memfile_to_ram, py::arg("ConnectNo"), py::arg("pbuffer"), py::arg("buffsize"), py::arg("filetype"));
    // m.def("smc_get_progress", &py_smc_get_progress, py::arg("ConnectNo"), py::arg("process"));
    // m.def("smc_udisk_get_state", &py_smc_udisk_get_state, py::arg("ConnectNo"), py::arg("state"));
    // m.def("smc_udisk_check_file", &py_smc_udisk_check_file, py::arg("ConnectNo"), py::arg("filename"), py::arg("filesize"), py::arg("filetype"));
    // m.def("smc_udisk_get_first_file", &py_smc_udisk_get_first_file, py::arg("ConnectNo"), py::arg("filename"), py::arg("filesize"), py::arg("fileid"), py::arg("filetype"));
    // m.def("smc_udisk_get_next_file", &py_smc_udisk_get_next_file, py::arg("ConnectNo"), py::arg("filename"), py::arg("filesize"), py::arg("fileid"), py::arg("filetype"));
    // m.def("smc_udisk_copy_file", &py_smc_udisk_copy_file, py::arg("ConnectNo"), py::arg("SrcFileName"), py::arg("DstFileName"), py::arg("filetype"), py::arg("mode"));
    // m.def("smc_set_modbus_0x", &py_smc_set_modbus_0x, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_get_modbus_0x", &py_smc_get_modbus_0x, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_set_modbus_4x", &py_smc_set_modbus_4x, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_get_modbus_4x", &py_smc_get_modbus_4x, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_set_modbus_4x_float", &py_smc_set_modbus_4x_float, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_get_modbus_4x_float", &py_smc_get_modbus_4x_float, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_set_modbus_4x_int", &py_smc_set_modbus_4x_int, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_get_modbus_4x_int", &py_smc_get_modbus_4x_int, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_set_persistent_reg", &py_smc_set_persistent_reg, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_get_persistent_reg", &py_smc_get_persistent_reg, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_set_persistent_reg_byte", &py_smc_set_persistent_reg_byte, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_get_persistent_reg_byte", &py_smc_get_persistent_reg_byte, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_set_persistent_reg_float", &py_smc_set_persistent_reg_float, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_get_persistent_reg_float", &py_smc_get_persistent_reg_float, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_set_persistent_reg_int", &py_smc_set_persistent_reg_int, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_get_persistent_reg_int", &py_smc_get_persistent_reg_int, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_set_persistent_reg_short", &py_smc_set_persistent_reg_short, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_get_persistent_reg_short", &py_smc_get_persistent_reg_short, py::arg("ConnectNo"), py::arg("start"), py::arg("inum"), py::arg("pdata"));
    // m.def("smc_read_array", &py_smc_read_array, py::arg("ConnectNo"), py::arg("name"), py::arg("index"), py::arg("var"), py::arg("num"));
    // m.def("smc_modify_array", &py_smc_modify_array, py::arg("ConnectNo"), py::arg("name"), py::arg("index"), py::arg("var"), py::arg("num"));
    // m.def("smc_read_var", &py_smc_read_var, py::arg("ConnectNo"), py::arg("varstring"), py::arg("var"), py::arg("num"));
    // m.def("smc_modify_var", &py_smc_modify_var, py::arg("ConnectNo"), py::arg("varstring"), py::arg("var"), py::arg("varnum"));
    // m.def("smc_write_array", &py_smc_write_array, py::arg("ConnectNo"), py::arg("name"), py::arg("startindex"), py::arg("var"), py::arg("num"));
    // m.def("smc_read_array_ex", &py_smc_read_array_ex, py::arg("ConnectNo"), py::arg("name"), py::arg("index"), py::arg("var"), py::arg("num"));
    // m.def("smc_modify_array_ex", &py_smc_modify_array_ex, py::arg("ConnectNo"), py::arg("name"), py::arg("index"), py::arg("var"), py::arg("num"));
    // m.def("smc_read_var_ex", &py_smc_read_var_ex, py::arg("ConnectNo"), py::arg("varstring"), py::arg("var"), py::arg("num"));
    // m.def("smc_modify_var_ex", &py_smc_modify_var_ex, py::arg("ConnectNo"), py::arg("varstring"), py::arg("var"), py::arg("varnum"));
    // m.def("smc_write_array_ex", &py_smc_write_array_ex, py::arg("ConnectNo"), py::arg("name"), py::arg("startindex"), py::arg("var"), py::arg("num"));
    // m.def("smc_get_stringtype", &py_smc_get_stringtype, py::arg("ConnectNo"), py::arg("varstring"), py::arg("m_Type"), py::arg("num"));
    // m.def("smc_basic_delete_file", &py_smc_basic_delete_file, py::arg("ConnectNo"));
    // m.def("smc_basic_run", &py_smc_basic_run, py::arg("ConnectNo"));
    // m.def("smc_basic_stop", &py_smc_basic_stop, py::arg("ConnectNo"));
    // m.def("smc_basic_pause", &py_smc_basic_pause, py::arg("ConnectNo"));
    // m.def("smc_basic_step_run", &py_smc_basic_step_run, py::arg("ConnectNo"));
    // m.def("smc_basic_step_over", &py_smc_basic_step_over, py::arg("ConnectNo"));
    // m.def("smc_basic_continue_run", &py_smc_basic_continue_run, py::arg("ConnectNo"));
    // m.def("smc_basic_state", &py_smc_basic_state, py::arg("ConnectNo"), py::arg("State"));
    // m.def("smc_basic_current_line", &py_smc_basic_current_line, py::arg("ConnectNo"), py::arg("line"));
    // m.def("smc_basic_break_info", &py_smc_basic_break_info, py::arg("ConnectNo"), py::arg("line"), py::arg("linenum"));
    // m.def("smc_basic_message", &py_smc_basic_message, py::arg("ConnectNo"), py::arg("pbuff"), py::arg("uimax"), py::arg("puiread"));
    // m.def("smc_basic_command", &py_smc_basic_command, py::arg("ConnectNo"), py::arg("pszCommand"), py::arg("psResponse"), py::arg("uiResponseLength"));
    // m.def("smc_gcode_check_file", &py_smc_gcode_check_file, py::arg("ConnectNo"), py::arg("pfilenameinControl"), py::arg("pbIfExist"), py::arg("pFileSize"));
    // m.def("smc_gcode_get_first_file", &py_smc_gcode_get_first_file, py::arg("ConnectNo"), py::arg("pfilenameinControl"), py::arg("pFileSize"));
    // m.def("smc_gcode_get_next_file", &py_smc_gcode_get_next_file, py::arg("ConnectNo"), py::arg("pfilenameinControl"), py::arg("pFileSize"));
    // m.def("smc_gcode_start", &py_smc_gcode_start, py::arg("ConnectNo"));
    // m.def("smc_gcode_stop", &py_smc_gcode_stop, py::arg("ConnectNo"));
    // m.def("smc_gcode_pause", &py_smc_gcode_pause, py::arg("ConnectNo"));
    // m.def("smc_gcode_state", &py_smc_gcode_state, py::arg("ConnectNo"), py::arg("State"));
    // m.def("smc_gcode_set_current_file", &py_smc_gcode_set_current_file, py::arg("ConnectNo"), py::arg("pFileName"));
    // m.def("smc_gcode_get_current_file", &py_smc_gcode_get_current_file, py::arg("ConnectNo"), py::arg("pfilenameinControl"), py::arg("fileid"));
    // m.def("smc_gcode_current_line", &py_smc_gcode_current_line, py::arg("ConnectNo"), py::arg("line"), py::arg("pCurLine"));
    // m.def("smc_gcode_get_current_line", &py_smc_gcode_get_current_line, py::arg("ConnectNo"), py::arg("line"), py::arg("pCurLine"));
    // m.def("smc_gcode_check_file_id", &py_smc_gcode_check_file_id, py::arg("ConnectNo"), py::arg("fileid"), py::arg("pFileName"), py::arg("pFileSize"), py::arg("pTotalLine"));
    // m.def("smc_gcode_check_file_name", &py_smc_gcode_check_file_name, py::arg("ConnectNo"), py::arg("pFileName"), py::arg("fileid"), py::arg("pFileSize"), py::arg("pTotalLine"));
    // m.def("smc_gcode_get_file_profile", &py_smc_gcode_get_file_profile, py::arg("ConnectNo"), py::arg("maxfilenum"), py::arg("maxfilesize"), py::arg("savedfilenum"));
    // m.def("smc_gcode_add_line", &py_smc_gcode_add_line, py::arg("ConnectNo"), py::arg("strline"));
    // m.def("smc_gcode_add_line_array", &py_smc_gcode_add_line_array, py::arg("ConnectNo"), py::arg("arraysize"), py::arg("linearray"));
    // m.def("smc_gcode_insert_line", &py_smc_gcode_insert_line, py::arg("ConnectNo"), py::arg("lineno"), py::arg("strline"));
    // m.def("smc_gcode_insert_line_array", &py_smc_gcode_insert_line_array, py::arg("ConnectNo"), py::arg("lineno"), py::arg("arraysize"), py::arg("linearray"));
    // m.def("smc_gcode_modify_line", &py_smc_gcode_modify_line, py::arg("ConnectNo"), py::arg("lineno"), py::arg("strline"));
    // m.def("smc_gcode_modify_line_array", &py_smc_gcode_modify_line_array, py::arg("ConnectNo"), py::arg("lineno"), py::arg("arraysize"), py::arg("linearray"));
    // m.def("smc_gcode_delete_line", &py_smc_gcode_delete_line, py::arg("ConnectNo"), py::arg("lineno"));
    // m.def("smc_gcode_get_line", &py_smc_gcode_get_line, py::arg("ConnectNo"), py::arg("line"), py::arg("strLine"));
    // m.def("smc_gcode_get_line_array", &py_smc_gcode_get_line_array, py::arg("ConnectNo"), py::arg("lineno"), py::arg("arraysize"), py::arg("linearray"));
    // m.def("smc_gcode_create_file", &py_smc_gcode_create_file, py::arg("ConnectNo"), py::arg("FileName"));
    // m.def("smc_gcode_save_file", &py_smc_gcode_save_file, py::arg("ConnectNo"), py::arg("FileName"));
    // m.def("smc_gcode_copy_file", &py_smc_gcode_copy_file, py::arg("ConnectNo"), py::arg("strFileName"), py::arg("newFileName"));
    // m.def("smc_gcode_rename_file", &py_smc_gcode_rename_file, py::arg("ConnectNo"), py::arg("strFileName"), py::arg("newFileName"));
    // m.def("smc_gcode_delete_fileid", &py_smc_gcode_delete_fileid, py::arg("ConnectNo"), py::arg("fileid"));
    // m.def("smc_gcode_delete_file", &py_smc_gcode_delete_file, py::arg("ConnectNo"), py::arg("pfilenameinControl"));
    // m.def("smc_gcode_clear_file", &py_smc_gcode_clear_file, py::arg("ConnectNo"));
    // m.def("smc_gcode_get_fileid", &py_smc_gcode_get_fileid, py::arg("ConnectNo"), py::arg("fileid"), py::arg("pFileName"), py::arg("filesize"));
    // m.def("smc_gcode_set_step_state", &py_smc_gcode_set_step_state, py::arg("ConnectNo"), py::arg("state"));
    // m.def("smc_gcode_get_step_state", &py_smc_gcode_get_step_state, py::arg("ConnectNo"), py::arg("state"));
    // m.def("smc_gcode_stop_reason", &py_smc_gcode_stop_reason, py::arg("ConnectNo"), py::arg("stop_reason"));
    m.def("smc_emg_stop", &py_smc_emg_stop, py::arg("ConnectNo"));
    m.def("smc_check_done", &py_smc_check_done, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_stop", &py_smc_stop, py::arg("ConnectNo"), py::arg("axis"), py::arg("stop_mode"));
    m.def("smc_check_done_multicoor", &py_smc_check_done_multicoor, py::arg("ConnectNo"), py::arg("Crd"));
    m.def("smc_stop_multicoor", &py_smc_stop_multicoor, py::arg("ConnectNo"), py::arg("Crd"), py::arg("stop_mode"));
    m.def("smc_axis_io_status", &py_smc_axis_io_status, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_axis_io_enable_status", &py_smc_axis_io_enable_status, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_get_axis_run_mode", &py_smc_get_axis_run_mode, py::arg("ConnectNo"), py::arg("axis"));
    m.def("smc_read_current_speed_unit", &py_smc_read_current_speed_unit, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_set_position_unit", &py_smc_set_position_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"));
    m.def("smc_get_position_unit", &py_smc_get_position_unit, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_get_target_position_unit", &py_smc_get_target_position_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"));
    // m.def("smc_set_workpos_unit", &py_smc_set_workpos_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"));
    // m.def("smc_get_workpos_unit", &py_smc_get_workpos_unit, py::arg("ConnectNo"), py::arg("axis"), py::arg("pos"));
    // m.def("smc_get_stop_reason", &py_smc_get_stop_reason, py::arg("ConnectNo"), py::arg("axis"), py::arg("StopReason"));
    m.def("smc_clear_stop_reason", &py_smc_clear_stop_reason, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_trace_set_source", &py_smc_trace_set_source, py::arg("ConnectNo"), py::arg("source"));
    // m.def("smc_read_trace_data", &py_smc_read_trace_data, py::arg("ConnectNo"), py::arg("axis"), py::arg("bufsize"), py::arg("time"), py::arg("pos"), py::arg("vel"), py::arg("acc"), py::arg("recv_num"));
    // m.def("smc_trace_start", &py_smc_trace_start, py::arg("ConnectNo"), py::arg("AxisNum"), py::arg("AxisList"));
    // m.def("smc_trace_stop", &py_smc_trace_stop, py::arg("ConnectNo"));
    // m.def("smc_trace_set_config", &py_smc_trace_set_config, py::arg("ConnectNo"), py::arg("trace_cycle"), py::arg("lost_handle"), py::arg("trace_type"), py::arg("trigger_object_index"), py::arg("trigger_type"), py::arg("mask"), py::arg("condition"));
    // m.def("smc_trace_get_config", &py_smc_trace_get_config, py::arg("ConnectNo"), py::arg("trace_cycle"), py::arg("lost_handle"), py::arg("trace_type"), py::arg("trigger_object_index"), py::arg("trigger_type"), py::arg("mask"), py::arg("condition"));
    // m.def("smc_trace_reset_config_object", &py_smc_trace_reset_config_object, py::arg("ConnectNo"));
    // m.def("smc_trace_add_config_object", &py_smc_trace_add_config_object, py::arg("ConnectNo"), py::arg("data_type"), py::arg("data_index"), py::arg("data_sub_index"), py::arg("slave_id"), py::arg("data_bytes"));
    // m.def("smc_trace_get_config_object", &py_smc_trace_get_config_object, py::arg("ConnectNo"), py::arg("object_index"), py::arg("data_type"), py::arg("data_index"), py::arg("data_sub_index"), py::arg("slave_id"), py::arg("data_bytes"));
    // m.def("smc_trace_data_start", &py_smc_trace_data_start, py::arg("ConnectNo"));
    // m.def("smc_trace_data_stop", &py_smc_trace_data_stop, py::arg("ConnectNo"));
    // m.def("smc_trace_data_reset", &py_smc_trace_data_reset, py::arg("ConnectNo"));
    // m.def("smc_trace_get_flag", &py_smc_trace_get_flag, py::arg("ConnectNo"), py::arg("start_flag"), py::arg("triggered_flag"), py::arg("lost_flag"));
    // m.def("smc_trace_get_state", &py_smc_trace_get_state, py::arg("ConnectNo"), py::arg("valid_num"), py::arg("free_num"), py::arg("object_total_bytes"), py::arg("object_total_num"));
    // m.def("smc_trace_get_data", &py_smc_trace_get_data, py::arg("ConnectNo"), py::arg("bufsize"), py::arg("data"), py::arg("byte_size"));
    // m.def("smc_trace_reset_lost_flag", &py_smc_trace_reset_lost_flag, py::arg("ConnectNo"));
    // m.def("nmcs_set_node_od", &py_nmcs_set_node_od, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeNo"), py::arg("Index"), py::arg("SubIndex"), py::arg("ValLength"), py::arg("Value"));
    // m.def("nmcs_get_node_od", &py_nmcs_get_node_od, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeNo"), py::arg("Index"), py::arg("SubIndex"), py::arg("ValLength"), py::arg("Value"));
    // m.def("nmcs_set_node_od_float", &py_nmcs_set_node_od_float, py::arg("ConnectNo"), py::arg("PortNum"), py::arg("NodeNum"), py::arg("Index"), py::arg("SubIndex"), py::arg("ValLength"), py::arg("Value"));
    // m.def("nmcs_get_node_od_float", &py_nmcs_get_node_od_float, py::arg("ConnectNo"), py::arg("PortNum"), py::arg("NodeNum"), py::arg("Index"), py::arg("SubIndex"), py::arg("ValLength"), py::arg("Value"));
    // m.def("nmcs_set_node_od_pbyte", &py_nmcs_set_node_od_pbyte, py::arg("ConnectNo"), py::arg("PortNum"), py::arg("NodeNum"), py::arg("Index"), py::arg("SubIndex"), py::arg("Bytes"), py::arg("Value"));
    // m.def("nmcs_get_node_od_pbyte", &py_nmcs_get_node_od_pbyte, py::arg("ConnectNo"), py::arg("PortNum"), py::arg("NodeNum"), py::arg("Index"), py::arg("SubIndex"), py::arg("Bytes"), py::arg("Value"));
    // m.def("nmcs_set_axis_enable", &py_nmcs_set_axis_enable, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("nmcs_set_axis_disable", &py_nmcs_set_axis_disable, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("nmcs_get_axis_io_out", &py_nmcs_get_axis_io_out, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("nmcs_set_axis_io_out", &py_nmcs_set_axis_io_out, py::arg("ConnectNo"), py::arg("axis"), py::arg("iostate"));
    // m.def("nmcs_get_axis_io_in", &py_nmcs_get_axis_io_in, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("nmcs_set_cycletime", &py_nmcs_set_cycletime, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("CycleTime"));
    // m.def("nmcs_get_cycletime", &py_nmcs_get_cycletime, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("CycleTime"));
    // m.def("nmcs_set_offset_pos", &py_nmcs_set_offset_pos, py::arg("ConnectNo"), py::arg("axis"), py::arg("offset_pos"));
    // m.def("nmcs_get_offset_pos", &py_nmcs_get_offset_pos, py::arg("ConnectNo"), py::arg("axis"), py::arg("offset_pos"));
    // m.def("nmcs_get_axis_type", &py_nmcs_get_axis_type, py::arg("ConnectNo"), py::arg("axis"), py::arg("Axis_Type"));
    // m.def("nmcs_axis_io_status", &py_nmcs_axis_io_status, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("nmcs_get_card_errcode", &py_nmcs_get_card_errcode, py::arg("ConnectNo"), py::arg("Errcode"));
    // m.def("nmcs_clear_card_errcode", &py_nmcs_clear_card_errcode, py::arg("ConnectNo"));
    // m.def("nmcs_get_errcode", &py_nmcs_get_errcode, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("Errcode"));
    // m.def("nmcs_clear_errcode", &py_nmcs_clear_errcode, py::arg("ConnectNo"), py::arg("PortNo"));
    // m.def("nmcs_get_axis_errcode", &py_nmcs_get_axis_errcode, py::arg("ConnectNo"), py::arg("axis"), py::arg("Errcode"));
    // m.def("nmcs_clear_axis_errcode", &py_nmcs_clear_axis_errcode, py::arg("ConnectNo"), py::arg("iaxis"));
    // m.def("nmcs_get_total_axes", &py_nmcs_get_total_axes, py::arg("ConnectNo"), py::arg("TotalAxis"));
    // m.def("nmcs_get_total_ionum", &py_nmcs_get_total_ionum, py::arg("ConnectNo"), py::arg("TotalIn"), py::arg("TotalOut"));
    // m.def("nmcs_read_inbit_extern", &py_nmcs_read_inbit_extern, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeID"), py::arg("BitNo"), py::arg("IoValue"));
    // m.def("nmcs_read_inport_extern", &py_nmcs_read_inport_extern, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeID"), py::arg("IoPortNo"), py::arg("IoValue"));
    // m.def("nmcs_write_outbit_extern", &py_nmcs_write_outbit_extern, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeID"), py::arg("BitNo"), py::arg("IoValue"));
    // m.def("nmcs_write_outport_extern", &py_nmcs_write_outport_extern, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeID"), py::arg("IoPortNo"), py::arg("IoValue"));
    // m.def("nmcs_read_outbit_extern", &py_nmcs_read_outbit_extern, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeID"), py::arg("BitNo"), py::arg("IoValue"));
    // m.def("nmcs_read_outport_extern", &py_nmcs_read_outport_extern, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeID"), py::arg("IoPortNo"), py::arg("IoValue"));
    // m.def("nmcs_set_slave_output_retain", &py_nmcs_set_slave_output_retain, py::arg("ConnectNo"), py::arg("Enable"));
    // m.def("nmcs_get_slave_output_retain", &py_nmcs_get_slave_output_retain, py::arg("ConnectNo"), py::arg("Enable"));
    // m.def("nmcs_reset_canopen", &py_nmcs_reset_canopen, py::arg("ConnectNo"));
    // m.def("nmcs_get_LostHeartbeat_Nodes", &py_nmcs_get_LostHeartbeat_Nodes, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeID"), py::arg("NodeNum"));
    // m.def("nmcs_get_EmergeneyMessege_Nodes", &py_nmcs_get_EmergeneyMessege_Nodes, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeMsg"), py::arg("MsgNum"));
    // m.def("nmcs_SendNmtCommand", &py_nmcs_SendNmtCommand, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeID"), py::arg("NmtCommand"));
    // m.def("nmcs_set_alarm_clear", &py_nmcs_set_alarm_clear, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("NodeNo"));
    // m.def("nmcs_syn_move_unit", &py_nmcs_syn_move_unit, py::arg("ConnectNo"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("Position"), py::arg("PosiMode"));
    // m.def("nmcs_get_total_adcnum", &py_nmcs_get_total_adcnum, py::arg("ConnectNo"), py::arg("TotalIn"), py::arg("TotalOut"));
    // m.def("nmcs_reset_etc", &py_nmcs_reset_etc, py::arg("ConnectNo"));
    // m.def("nmcs_stop_etc", &py_nmcs_stop_etc, py::arg("ConnectNo"), py::arg("ETCState"));
    // m.def("nmcs_get_axis_state_machine", &py_nmcs_get_axis_state_machine, py::arg("ConnectNo"), py::arg("axis"), py::arg("Axis_StateMachine"));
    // m.def("nmcs_get_axis_node_address", &py_nmcs_get_axis_node_address, py::arg("ConnectNo"), py::arg("axis"), py::arg("SlaveAddr"), py::arg("Sub_SlaveAddr"));
    // m.def("nmcs_write_rxpdo_extra", &py_nmcs_write_rxpdo_extra, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("address"), py::arg("DataLen"), py::arg("Value"));
    // m.def("nmcs_read_rxpdo_extra", &py_nmcs_read_rxpdo_extra, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("address"), py::arg("DataLen"), py::arg("Value"));
    // m.def("nmcs_read_txpdo_extra", &py_nmcs_read_txpdo_extra, py::arg("ConnectNo"), py::arg("PortNo"), py::arg("address"), py::arg("DataLen"), py::arg("Value"));
    // m.def("nmcs_torque_move", &py_nmcs_torque_move, py::arg("CardNo"), py::arg("axis"), py::arg("Torque"), py::arg("PosLimitValid"), py::arg("PosLimitValue"), py::arg("PosMode"));
    // m.def("nmcs_change_torque", &py_nmcs_change_torque, py::arg("CardNo"), py::arg("axis"), py::arg("Torque"));
    // m.def("nmcs_get_torque", &py_nmcs_get_torque, py::arg("CardNo"), py::arg("axis"), py::arg("Torque"));
    // m.def("smc_pdo_buffer_enter", &py_smc_pdo_buffer_enter, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_pdo_buffer_stop", &py_smc_pdo_buffer_stop, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_pdo_buffer_clear", &py_smc_pdo_buffer_clear, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("smc_pdo_buffer_run_state", &py_smc_pdo_buffer_run_state, py::arg("ConnectNo"), py::arg("axis"), py::arg("RunState"), py::arg("Remain"), py::arg("NotRunned"), py::arg("Runned"));
    // m.def("smc_pdo_buffer_add_data", &py_smc_pdo_buffer_add_data, py::arg("ConnectNo"), py::arg("axis"), py::arg("size"), py::arg("data_table"));
    // m.def("smc_pdo_buffer_start_multi", &py_smc_pdo_buffer_start_multi, py::arg("ConnectNo"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("ResultList"));
    // m.def("smc_pdo_buffer_pause_multi", &py_smc_pdo_buffer_pause_multi, py::arg("ConnectNo"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("ResultList"));
    // m.def("smc_pdo_buffer_stop_multi", &py_smc_pdo_buffer_stop_multi, py::arg("ConnectNo"), py::arg("AxisNum"), py::arg("AxisList"), py::arg("ResultList"));
    // m.def("nmcs_start_pdo_trace", &py_nmcs_start_pdo_trace, py::arg("ConnectNo"), py::arg("Channel"), py::arg("SlaveAddr"), py::arg("Index_Num"), py::arg("Trace_Len"), py::arg("Index"), py::arg("Sub_Index"));
    // m.def("nmcs_get_pdo_trace", &py_nmcs_get_pdo_trace, py::arg("ConnectNo"), py::arg("Channel"), py::arg("SlaveAddr"), py::arg("Index_Num"), py::arg("Trace_Len"), py::arg("Index"), py::arg("Sub_Index"));
    // m.def("nmcs_set_pdo_trace_trig_para", &py_nmcs_set_pdo_trace_trig_para, py::arg("ConnectNo"), py::arg("Channel"), py::arg("SlaveAddr"), py::arg("Trig_Index"), py::arg("Trig_Sub_Index"), py::arg("Trig_Value"), py::arg("Trig_Mode"));
    // m.def("nmcs_get_pdo_trace_trig_para", &py_nmcs_get_pdo_trace_trig_para, py::arg("ConnectNo"), py::arg("Channel"), py::arg("SlaveAddr"), py::arg("Trig_Index"), py::arg("Trig_Sub_Index"), py::arg("Trig_Value"), py::arg("Trig_Mode"));
    // m.def("nmcs_clear_pdo_trace_data", &py_nmcs_clear_pdo_trace_data, py::arg("ConnectNo"), py::arg("Channel"), py::arg("SlaveAddr"));
    // m.def("nmcs_stop_pdo_trace", &py_nmcs_stop_pdo_trace, py::arg("ConnectNo"), py::arg("Channel"), py::arg("SlaveAddr"));
    // m.def("nmcs_read_pdo_trace_data", &py_nmcs_read_pdo_trace_data, py::arg("ConnectNo"), py::arg("Channel"), py::arg("SlaveAddr"), py::arg("StartAddr"), py::arg("Readlen"), py::arg("ActReadlen"), py::arg("Data"));
    // m.def("nmcs_get_pdo_trace_num", &py_nmcs_get_pdo_trace_num, py::arg("ConnectNo"), py::arg("Channel"), py::arg("SlaveAddr"), py::arg("Data_num"), py::arg("Size_of_each_bag"));
    // m.def("nmcs_get_pdo_trace_state", &py_nmcs_get_pdo_trace_state, py::arg("ConnectNo"), py::arg("Channel"), py::arg("SlaveAddr"), py::arg("Trace_state"));
    // m.def("nmcs_get_current_fieldbus_state_info", &py_nmcs_get_current_fieldbus_state_info, py::arg("ConnectNo"), py::arg("Channel"), py::arg("Axis"), py::arg("ErrorType"), py::arg("SlaveAddr"), py::arg("ErrorFieldbusCode"));
    // m.def("nmcs_get_detail_fieldbus_state_info", &py_nmcs_get_detail_fieldbus_state_info, py::arg("ConnectNo"), py::arg("Channel"), py::arg("ReadErrorNum"), py::arg("TotalNum"), py::arg("ActualNum"), py::arg("Axis"), py::arg("ErrorType"), py::arg("SlaveAddr"), py::arg("ErrorFieldbusCode"));
    // m.def("nmcs_reset_rtex", &py_nmcs_reset_rtex, py::arg("ConnectNo"));
    // m.def("nmcs_start_connect", &py_nmcs_start_connect, py::arg("ConnectNo"), py::arg("chan"), py::arg("info"), py::arg("len"));
    // m.def("nmcs_get_vendor_info", &py_nmcs_get_vendor_info, py::arg("ConnectNo"), py::arg("axis"), py::arg("info"), py::arg("len"));
    // m.def("nmcs_get_slave_type_info", &py_nmcs_get_slave_type_info, py::arg("ConnectNo"), py::arg("axis"), py::arg("info"), py::arg("len"));
    // m.def("nmcs_get_slave_name_info", &py_nmcs_get_slave_name_info, py::arg("ConnectNo"), py::arg("axis"), py::arg("info"), py::arg("len"));
    // m.def("nmcs_get_slave_version_info", &py_nmcs_get_slave_version_info, py::arg("ConnectNo"), py::arg("axis"), py::arg("info"), py::arg("len"));
    // m.def("nmcs_write_parameter", &py_nmcs_write_parameter, py::arg("ConnectNo"), py::arg("axis"), py::arg("index"), py::arg("subindex"), py::arg("para_data"));
    // m.def("nmcs_write_slave_eeprom", &py_nmcs_write_slave_eeprom, py::arg("ConnectNo"), py::arg("axis"));
    // m.def("nmcs_read_parameter", &py_nmcs_read_parameter, py::arg("ConnectNo"), py::arg("axis"), py::arg("index"), py::arg("subindex"), py::arg("para_data"));
    // m.def("nmcs_read_parameter_attributes", &py_nmcs_read_parameter_attributes, py::arg("ConnectNo"), py::arg("axis"), py::arg("index"), py::arg("subindex"), py::arg("para_data"));
    // m.def("nmcs_set_cmdcycletime", &py_nmcs_set_cmdcycletime, py::arg("ConnectNo"), py::arg("PortNum"), py::arg("cmdtime"));
    // m.def("nmcs_get_cmdcycletime", &py_nmcs_get_cmdcycletime, py::arg("ConnectNo"), py::arg("PortNum"), py::arg("cmdtime"));
    // m.def("nmcs_start_log", &py_nmcs_start_log, py::arg("ConnectNo"), py::arg("chan"), py::arg("node"), py::arg("Ifenable"));
    // m.def("nmcs_get_log", &py_nmcs_get_log, py::arg("ConnectNo"), py::arg("chan"), py::arg("node"), py::arg("data"));
    // m.def("nmcs_config_atuo_log", &py_nmcs_config_atuo_log, py::arg("ConnectNo"), py::arg("ifenable"), py::arg("dir"), py::arg("byte_index"), py::arg("mask"), py::arg("condition"), py::arg("counter"));
    // m.def("nmcs_get_log_state", &py_nmcs_get_log_state, py::arg("ConnectNo"), py::arg("chan"), py::arg("state"));
    // m.def("nmcs_driver_reset", &py_nmcs_driver_reset, py::arg("ConnectNo"), py::arg("axis"));

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}