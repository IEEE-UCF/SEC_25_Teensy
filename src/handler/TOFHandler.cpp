#include "TOFHandler.h"

TOFHandler::TOFHandler(int *cToFs, int numChannels)
{
  this->cToFs = cToFs;
  this->numChannels = numChannels;
  sensors = new VL53L0X[numChannels];
  distances = new int[numChannels];
}

TOFHandler::~TOFHandler()
{
  delete[] sensors;
  delete[] distances;
}

void TOFHandler::Begin()
{
  for (int i = 0; i < numChannels; i++)
  {
    i2cmux::tcaselect(cToFs[i]);
    sensors[i].setBus(&Wire1);
    // sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print(F("Failed to detect and initialize sensor at channel "));
      Serial.println(cToFs[i]);
    }
    sensors[i].setSignalRateLimit(0.10);
    sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensors[i].startContinuous();
  }
}

void TOFHandler::Update()
{
  for (int i = 0; i < numChannels; i++)
  {
    i2cmux::tcaselect(cToFs[i]);
    distances[i] = sensors[i].readRangeContinuousMillimeters();
    if (sensors[i].timeoutOccurred())
    {
      Serial.print(F("Timeout occurred at channel "));
      Serial.println(cToFs[i]);
    }
  }
}

const int *TOFHandler::Get() const
{
  return distances;
}

int TOFHandler::GetIndex(int index) const
{
  if (index >= 0 && index < numChannels)
  {
    return distances[index];
  }
  return -1; // Return -1 if index is out of bounds
}

void TOFHandler::PrintInfo(Print &output, bool printConfig) const
{
  if (printConfig)
  {
    output.print(F("TOFHandler Configuration: "));
    output.print(F("Number of Channels: "));
    output.println(numChannels);
    output.print(F("Channels: "));
    for (int i = 0; i < numChannels; i++)
    {
      output.print(cToFs[i]);
      if (i < numChannels - 1)
      {
        output.print(F(", "));
      }
    }
    output.println();
  }
  else
  {
    output.print(F("Distances: "));
    for (int i = 0; i < numChannels; i++)
    {
      output.print(distances[i]);
      if (i < numChannels - 1)
      {
        output.print(F(", "));
      }
    }
    output.println();
  }
}

Print &operator<<(Print &output, const TOFHandler &handler)
{
  handler.PrintInfo(output, false);
  return output;
}