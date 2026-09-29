#include "neblib/devices/coprocessor.hpp"

neblib::Teensy::Teensy(uint32_t index)
    : serial_link(index, "name", vex::linkType::raw, true), // name is irrelavent, has to be wired for raw linktype
      mutex()
{
    baud(115200);
}

int32_t neblib::Teensy::sendReceive(
    uint8_t *sendBuffer,
    int32_t sendLength,
    uint8_t *readBuffer,
    int32_t readLength,
    int32_t timeout)
{
    mutex.lock(); // This will wait for the mutex to be unlocked if it is locked by another thread

    send(sendBuffer, sendLength);
    int32_t result = receive(readBuffer, readLength, timeout);

    mutex.unlock();
    return result;
}
