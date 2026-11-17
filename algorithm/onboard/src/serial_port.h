#ifndef _ONBOARD_SERIAL_PORT_H_
#define _ONBOARD_SERIAL_PORT_H_

// POSIX 串口收发：读线程把收到的字节攒进缓冲，主循环定期取走。
// 用途：机载计算机用 USB 串口直连飞控（硬件在环 / 实飞）。
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ego_onboard
{

  class SerialPort
  {
  public:
    ~SerialPort();
    bool open(const std::string &dev, int baud, std::string &err);
    void close();
    bool isOpen() const { return fd_ >= 0 && run_; }
    void take(std::vector<uint8_t> &out);
    bool write(const uint8_t *data, size_t len);

  private:
    int fd_ = -1;
    std::atomic<bool> run_{false};
    std::thread reader_;
    std::mutex mtx_;
    std::vector<uint8_t> rx_;
  };

} // namespace ego_onboard

#endif
