#ifndef CANBUS_IDS_H
#define CANBUS_IDS_H
#include "string"
#include "map"
#include <cstdint>

class PrinterCANBus
{
private:
    std::map<std::string, uint32_t> m_ids;
public:
    PrinterCANBus(std::string section_name);
    ~PrinterCANBus();
    uint32_t add_uuid(std::string canbus_uuid, std::string canbus_iface);
    uint32_t get_nodeid(std::string canbus_uuid);
};




#endif