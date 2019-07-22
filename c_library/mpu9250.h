#ifndef MPU9250_H
#define MPU9250_H

class MPU9250 {
private:
  char _addr;

public:
  MPU9250(char addr);

  void readData();
  void setUpRegisters();
}





#endif //MPU9250_H
