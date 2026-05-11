#ifndef SERIALHDL_H
#define SERIALHDL_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <mutex>
#include <functional>
#include "msgproto.h"
#include <condition_variable>

extern "C"
{
#include "../chelper/serialqueue.h"
#include "../chelper/pyhelper.h"
}

struct msgParams
{
    std::string name;
    double data;
    int offset;
    bool is_empty;
};

struct ConstantWapper
{
    int receive_window;      // RECEIVE_WINDOW
    double clock_freq;       // CLOCK_FREQ
    double serial_baud;      // SERIAL_BAUD
    double pwm_max;          // PWM_MAX
    double adc_max;          // ADC_MAX
    double stats_sumsq_base; // STATS_SUMSQ_BASE
};

class Serialhdl
{
public:
    Serialhdl(int pin32, std::string name);
    ~Serialhdl();

    std::string m_name;
    pthread_t serial_tid;
    std::mutex m_mtx;
    bool m_is_active;
    std::map<std::string, std::function<void(ParseResult &)>> m_response_callbacks;

    MessageParser *m_msgparser;
    command_queue *m_command_queue;
    serialqueue *m_serialqueue;
    uint64_t m_last_notify_id;
    int fd_serial;
    void disconnect();
    int pins_per_bank;

    bool get_active();
    void register_response(std::function<void(ParseResult &)> response_callback, std::string name, int oid = 0);
    int serial_init(std::string serialport);
    void creat_serial_pthread(std::string serialport, uint32_t baud);
    command_queue *alloc_command_queue();
    MessageParser *get_msgparser();
    int connect_uart(std::string serialport, uint32_t baud, uint32_t mcu_type, int max_pending_blocks);
    std::string get_identify_data(double eventtime);
    void raw_send_wait_ack(uint8_t *cmd, int cmd_len, uint64_t minclock, uint64_t reqclock, command_queue *command_queue);
    void raw_send(uint8_t *cmd, int cmd_len, uint64_t minclock, uint64_t reqclock, command_queue *command_queue);
    ParseResult send_with_response(std::string msg, std::string response, command_queue *cmd_queue, int oid = 0, uint64_t minclock = 0, uint64_t reqclock = 0);
    void send(std::string msg, uint64_t minclock, uint64_t reqclock, command_queue *cmd_queue);
    ConstantWapper *get_constant_wapper() { return &constant_wapper; }
    std::string stats(double eventtime);

private:
    ConstantWapper constant_wapper;
};

class SerialRetryCommand
{
public:
    SerialRetryCommand(Serialhdl *serial, std::string name, int oid = 0);
    ~SerialRetryCommand();

    void handle_callback(ParseResult &params);
    ParseResult get_response(std::string& msg, uint8_t *cmds, int cmd_len, command_queue *command_queue, uint64_t minclock = 0, uint64_t reqclock = 0);

private:
    Serialhdl *m_serial;
    int m_oid;
    std::string m_name;
    ParseResult m_last_params;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_response_ready = false;
};

#endif