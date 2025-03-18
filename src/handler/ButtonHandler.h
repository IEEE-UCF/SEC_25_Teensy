#ifndef ButtonHandler_h
#define ButtonHandler_h

#include <Arduino.h>

class ButtonHandler
{
public:
  ButtonHandler(int *kPins, int numPins);
  void Begin();
  void Update();
  bool *buttonStates;

  void PrintInfo(Print &output, bool printConfig) const;
  friend Print &operator<<(Print &output, const ButtonHandler &handler);

private:
  int *kPins;
  int numPins;
};

#endif