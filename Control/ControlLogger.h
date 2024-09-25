#pragma once

#include <string>

class ControlLogger
{
public:
    static ControlLogger* getInstance()
    {
        if (!_instance)
            _instance = new ControlLogger();

        return _instance;
    }

    // Print/clear status & error messages for interact perceive
    void print_ip_status(std::string status);
    void clear_ip_status();
    void print_ip_info(std::string info);
    void print_ip_error(std::string error);
    void clear_ip_error();

private:

    static ControlLogger* _instance;   // The one, single instance
    ControlLogger() {} // private constructor
    ControlLogger(const ControlLogger&);
    ControlLogger& operator=(const ControlLogger&);

};