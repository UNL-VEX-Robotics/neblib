#include "neblib/devices/coprocessor.hpp"

neblib::Coprocessor::Coprocessor(uint32_t index)
    : serial_link(index, "name", vex::linkType::raw, true), // name is irrelavent, has to be wired for raw linktype
      mutex()
{
    baud(115200);
}

int32_t neblib::Coprocessor::sendReceive(
    uint8_t *sendBuffer,
    int32_t sendLength,
    uint8_t *readBuffer,
    int32_t readLength,
    int32_t timeout)
{
    mutex.lock(); // This will wait for the mutex to be unlocked if it is locked by another thread

    int32_t bytesSent = vex::serial_link::send(sendBuffer, sendLength);
    if (bytesSent != sendLength) // send was not complete
    {
        mutex.unlock();
        return -1;
    }
    int32_t result = receive(readBuffer, readLength, timeout);

    mutex.unlock();
    return result;
}

bool neblib::Coprocessor::ping(int32_t timeout)
{
    uint8_t sendBuffer[] = {PING};
    uint8_t receiveBuffer[1]; // todo: Test if this can work with 1 byte buffer

    int32_t receivedCount = this->sendReceive(sendBuffer, sizeof(sendBuffer), receiveBuffer, sizeof(receiveBuffer), timeout);

    if (receivedCount != 1)
        return false;

    if (receiveBuffer[0] == SUCCESS)
        return true;
    else
        return false; 
}

bool neblib::Coprocessor::send(uint8_t *sendBuffer, int32_t sendLength, int32_t timeout)
{
    uint8_t receiveBuffer[1];
    int32_t result = this->sendReceive(sendBuffer, sendLength, receiveBuffer, sizeof(receiveBuffer), timeout);

    if (result != 1)
        return false;
    else if (receiveBuffer[0] == Result::SUCCESS)
        return true;
    else
        return false;
}
