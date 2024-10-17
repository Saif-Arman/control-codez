#include "ControlLogger.h"
#include "Functions.h"

ControlLogger* ControlLogger::_instance = nullptr;

void ControlLogger::print_ip_status(std::string status)
{
	clear_ip_status();
	gotoxy(1, 46);
	std::cout << status;;
}

void ControlLogger::clear_ip_status()
{
	gotoxy(1, 46);
	std::cout << "\r                                                               \r";
}

void ControlLogger::print_ip_info(std::string info)
{
	print_ip_error(info);
}

void ControlLogger::print_ip_error(std::string error)
{
	clear_ip_error();
	gotoxy(1, 44);
	std::cout << error;
}

void ControlLogger::clear_ip_error()
{
	gotoxy(1, 44);
	std::cout << "\r                                                               \r";
}