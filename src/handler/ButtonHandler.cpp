#include "ButtonHandler.h"
#include <Arduino.h>

ButtonHandler::ButtonHandler(int *kPins, int numPins)
{
  this->kPins = kPins;
  this->numPins = numPins;
  buttonStates = new bool[numPins];
}

void ButtonHandler::Begin()
{
  for (int i = 0; i < numPins; i++)
  {
    pinMode(kPins[i], INPUT_PULLUP);
  }
}

void ButtonHandler::Update()
{
  for (int i = 0; i < numPins; i++)
  {
    buttonStates[i] = digitalRead(kPins[i]) == LOW;
  }
}

void ButtonHandler::PrintInfo(Print &output, bool printConfig) const
{
  if (printConfig)
  {
    output.print(F("ButtonHandler Configuration: "));
    output.print(F("Number of Pins: "));
    output.println(numPins);
    output.print(F("Pins: "));
    for (int i = 0; i < numPins; i++)
    {
      output.print(kPins[i]);
      if (i < numPins - 1)
      {
        output.print(F(", "));
      }
    }
    output.println();
  }
  else
  {
    output.print(F("Button States: "));
    for (int i = 0; i < numPins; i++)
    {
      output.print(buttonStates[i] ? F("Pressed") : F("Released"));
      if (i < numPins - 1)
      {
        output.print(F(", "));
      }
    }
    output.println();
  }
}

Print &operator<<(Print &output, const ButtonHandler &handler)
{
  handler.PrintInfo(output, false);
  return output;
}