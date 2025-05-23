#include "get_ip.hpp"

std::string exec(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

std::string getIPAddress(const std::string &interface)
{
    std::string cmd = "ifconfig " + interface + " | grep 'inet ' | awk '{print $2}'";
    std::string output = exec(cmd.c_str());

    if (!output.empty())
    {
        // Remove newline character from the end
        output.pop_back();
        return output;
    }
    return "";
}