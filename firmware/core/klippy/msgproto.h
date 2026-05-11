#ifndef __MSGPROTO_H__
#define __MSGPROTO_H__

#include <string>
#include <iostream> 
#include <list>
#include <vector>
#include <unordered_map>
#include <map>
#include <stdlib.h>
#include <cstdint>
#define MESSAGE_HEADER_SIZE 2

struct Msg
{
    std::string name;
    std::string value;
};

struct ParseResult
{
    std::string msg_name;
    double sent_time;
    double receive_time;
    std::map<std::string, uint32_t> PT_uint32_outs;
    std::map<std::string, std::string> PT_string_outs;
};

struct PT_uint32_OutParams{
    uint32_t v;
    int pos;
};

struct PT_string_OutParams{
    std::string v;
    int pos;
};
struct MsgType
{
    std::string name;
    std::string parse_type;
};

struct Params
{
    std::string name;
    double data;
    int offset;
};

struct Message{
    int msgtag;
    std::string mssgtype;
    std::string msgformat;
};

class MessageParser{
public:
    MessageParser(int pin32);
    ~MessageParser();
    int pins_per_bank;

    void init_messages_format_to_id_map(std::map<std::string, int> &messages_format);
    void init_id_to_messages_format_map(std::map<int, std::string> &messages_format);
    void init_messages_format_to_id_map_encode(std::map<std::string, int> &messages_format);
    void init_message_name_toId(std::map<std::string, int> &Message_by_name);
    void init_pin_map(std::map<std::string, int> &pinMap);
    void init_shutdown_info_map(std::map<int, std::string> &shutdown_info);
    void encode_int(std::vector<uint8_t> &out, uint32_t v);
    
    PT_uint32_OutParams PT_uint32_parse(uint8_t *s, int pos);
    PT_string_OutParams PT_string_parse(uint8_t *s, int pos);
    ParseResult parse(uint8_t* s);
    std::vector<uint8_t> create_command(std::string msg, std::map<std::string, int> message_by_name);

    std::map<int, std::string> m_id_to_format;
    std::map<std::string, int> m_format_to_id;
    std::map<std::string, int> m_namae_to_id;
    std::map<std::string, int> m_format_to_id_encode;
    std::map<std::string, int> m_pinMap;
    std::map<int, std::string> m_shutdown_info;

};
#endif
