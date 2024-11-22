#include "ControlLogger.h"
#include "Functions.h"

ControlLogger* ControlLogger::_instance = nullptr;

void ControlLogger::clear_line()
{
	std::cout << "\r                                                                                                                                  \r";
	return;
}

void ControlLogger::print_ip_status(std::string status)
{
	clear_ip_status();
	gotoxy(1, 46);
	std::cout << "IP Status: " << status;
	return;
}

void ControlLogger::clear_ip_status()
{
	gotoxy(1, 46);
	clear_line();
	return;
}

void ControlLogger::print_ip_info(std::string info)
{
	clear_ip_info();
	gotoxy(1, 45);
	std::cout << "IP Info: " << info << std::endl;
	return;
}

void ControlLogger::clear_ip_info()
{
	gotoxy(1, 45);
	clear_line();
	return;
}

void ControlLogger::print_ip_error(std::string error)
{
	clear_ip_error();
	gotoxy(1, 44);
	std::cout << "IP Error: " << error << std::endl;
	return;
}

void ControlLogger::clear_ip_error()
{
	gotoxy(1, 44);
	clear_line();
	return;
}