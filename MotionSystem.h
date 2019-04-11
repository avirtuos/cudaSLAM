#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

using namespace std;

class MotionSystem
{

public:
    MotionSystem(char dest[18]);
    ~MotionSystem();
    void forward();
    void backward();
    void left();
    void right();
    void stop();

private:
    int client_sd;
    bool enabled;

};

MotionSystem::MotionSystem(char dest[18])
{
    enabled = true;

    if(!enabled){return;}
    struct sockaddr_rc addr = { 0 };
    int status;
    //char dest[18] = "00:1B:10:80:13:ED";

    // allocate a socket
    client_sd = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );

    // connect to server
    status = connect(client_sd, (struct sockaddr *)&addr, sizeof(addr));

    // send a message
    if ( status < 0 ) {
        perror("Failed to connect to Bluetooth device.");
    }   
}

MotionSystem::~MotionSystem()
{
    if(!enabled){return;}
    close(client_sd);
}

void MotionSystem::stop()
{
    if(!enabled){return;}
    int status = write(client_sd, " ", 1);
    if(status < 0) {
        perror("Failed to send 'stop' directive to MotionSystem via bluetooth.");
    }
}

void MotionSystem::forward()
{
    if(!enabled){return;}
    int status = write(client_sd, "w", 1);
    if(status < 0) {
        perror("Failed to send 'forward' directive to MotionSystem via bluetooth.");
    }
}

void MotionSystem::backward()
{
    if(!enabled){return;}
    int status = write(client_sd, "s", 1);
    if(status < 0) {
        perror("Failed to send 'backward' directive to MotionSystem via bluetooth.");
    }
}

void MotionSystem::left()
{
    if(!enabled){return;}
    int status = write(client_sd, "a", 1);
    if(status < 0) {
        perror("Failed to send 'left' directive to MotionSystem via bluetooth.");
    }
}

void MotionSystem::right()
{
    if(!enabled){return;}
    int status = write(client_sd, "d", 1);
    if(status < 0) {
        perror("Failed to send 'right' directive to MotionSystem via bluetooth.");
    }
}