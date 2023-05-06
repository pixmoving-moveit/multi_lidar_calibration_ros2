#pragma once

#include <iostream>

#define RS_ERROR   std::cout << "\033[1m\033[31m"  // bold red
#define RS_WARNING std::cout << "\033[1m\033[33m"  // bold yellow
#define RS_INFO    std::cout << "\033[1m\033[32m"  // bold green
#define RS_INFOL   std::cout << "\033[32m"         // green
#define RS_DEBUG   std::cout << "\033[1m\033[36m"  // bold cyan
#define RS_REND    "\033[0m" << std::endl

#define RS_TITLE   std::cout << "\033[1m\033[35m"  // bold magenta
#define RS_MSG     std::cout << "\033[1m\033[37m"  // bold white