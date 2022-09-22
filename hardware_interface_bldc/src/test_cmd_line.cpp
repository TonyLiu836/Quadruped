#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

void open_port(std::string can_id, int bitrate){
    std::string set_bitrate = "sudo ip link set " + can_id + " type can bitrate " + std::to_string(bitrate);
    std::string can_up = "sudo ip link set up " + can_id;
    system(set_bitrate.c_str());
    system(can_up.c_str());
    //system("sudo ip link set " << can_id << " type can bitrate " << std::to_string(bitrate));
    //system("sudo ip link set up " << can_id);
    std::cout << "port opened @" << can_id << " with bitrate: " << bitrate << std::endl;
}

void close_port(std::string can_id){
    std::string close_port = "sudo ifconfig " + can_id + " down";
    system(close_port.c_str());
    std::cout << "port " << can_id << " closed" << std::endl;
}

int main(){
    //const char* canNum = "can0";
    open_port("can0", 1000000);
    
    close_port("can0");
    return 0;

}
