#include "SorterSubsystem.h"

SorterSubsystem::SorterSubsystem(int iTOF, int hallCount, int iServo, TOFHandler &tofs, HallHandler &halls, ServoHandler &servos, DriveMotor &transferMotor, RGBHandler &rgb)
    : iTOF(iTOF),
      hallCount(hallCount),
      iServo(iServo),
      tofs(tofs),
      halls(halls),
      servos(servos),
      transferMotor(transferMotor),
      rgb(rgb),
      _state(0),
      _baseReadings(nullptr),
      objectMagnet(false) {}

void SorterSubsystem::Begin()
{
    halls.Update();
    int *currentReadings = halls.getReadings();
    if (_baseReadings == nullptr)
    {
        _baseReadings = new int[hallCount];
    }

    for (int i = 0; i < hallCount; i++)
    {
        _baseReadings[i] = currentReadings[i];
    }
    _state = 5;
    MoveCenter();
}
/*
State 0:
No object detcted. Run transferMotor

State 1:
Object detected, wait for stabilization.

State 2:
Object detected, detect object type

State 3:
Operate servo until object is out of the way

State 4:
Wait a bit to stabilize

State 5:
Go to 0, stabilize
*/

/**
 * Updates, run every frame
 */
void SorterSubsystem::Update()
{
    using namespace GlobalColors;
    static elapsedMillis timer = 0;
    switch (_state)
    {
    case 0: // no object detected yet.
    {
        rgb.setSectionPulseEffect(5, PURPLE, 20);
        rgb.setSectionPulseEffect(6, PURPLE, 20);
        if (timer < 2000)
        {
            transferMotor.Set(150); // funnel into sorter
        }
        else if (timer < 2500)
        {
            transferMotor.Set(-100); // clear jams
        }
        else
        {
            timer = 0;
        }
        MoveCenter();                    // write center angle
        int range = tofs.GetIndex(iTOF); // get range reading from tof
        if (range < OBJECT_RANGE)
        {
            _state = 1; // if object is detcted, switch state
            timer = 0;
        }
        break;
    }

    case 1: // object detected. wait a bit
    {
        // Object newly detected
        rgb.stopSectionEffect(5);
        rgb.stopSectionEffect(6);
        transferMotor.Set(0); // pause transfer
        if (timer > 500)
        {
            _state = 2;
            timer = 0;
        }
        break;
    }

    case 2: // object detection, write the correcct servo angle.
    {
        if (timer > 150)
        {
            objectMagnet = false; // object does not have a magnet
            halls.Update();
            int *_readings = halls.getReadings(); // get halls readings
            for (int i = 0; i < hallCount; i++)
            {
                if (abs(_readings[i] - _baseReadings[i]) > BOUNDS_MAG)
                {
                    objectMagnet = true; // object does have a magnet
                }
                /*Serial.println(abs(_readings[i] - _baseReadings[i]));
                delay(500); debug */
            }
            if (objectMagnet)
            {
                MoveLeft();
                rgb.setSectionSolidColor(6, PURPLE);
            }
            else
            {
                MoveRight();
                rgb.setSectionSolidColor(5, PURPLE);
            }
            objectMagnet ? MoveLeft() : MoveRight();
            _state = 3;
            timer = 0;
            break;
        }
    }

    case 3: // write the correct servo angle. If object leaves, then move on
    {
        int range = tofs.GetIndex(iTOF);
        if (timer > 200)
        {
            objectMagnet ? MoveSoftLeft() : MoveSoftRight();
        }
        if (range > OBJECT_RANGE || timer > 500)
        {
            _state = 4;
            timer = 0;
            objectMagnet = false;
        }
    }
    break;

    case 4: // wait a bit for object to fall out
    {
        if (timer > 250)
        {
            _state = 5;
            timer = 0;
        }
        break;
    }

    case 5: // stabilize before running vl53l0x again
    {
        rgb.setSectionPulseEffect(5, 170, 0, 255, 20);
        rgb.setSectionPulseEffect(6, 170, 0, 255, 20);
        MoveCenter();
        if (timer > 250)
        {
            _state = 0;
            timer = 0;
        }
        break;
    }
    }
}

void SorterSubsystem::MoveCenter()
{
    servos.WriteServoAngle(iServo, SorterSubsystem::ServoPositions::CENTER);
}

void SorterSubsystem::MoveLeft()
{
    servos.WriteServoAngle(iServo, SorterSubsystem::ServoPositions::LEFT);
}

void SorterSubsystem::MoveRight()
{
    servos.WriteServoAngle(iServo, SorterSubsystem::ServoPositions::RIGHT);
}

void SorterSubsystem::MoveSoftLeft()
{
    servos.WriteServoAngle(iServo, SorterSubsystem::ServoPositions::SOFTLEFT);
}

void SorterSubsystem::MoveSoftRight()
{
    servos.WriteServoAngle(iServo, SorterSubsystem::ServoPositions::SOFTRIGHT);
}

void SorterSubsystem::PrintInfo(Print &output, bool printConfig) const
{
    // Print base readings (included in both sections)
    output.print(F("Base Hall Readings: "));
    if (_baseReadings != nullptr)
    {
        for (int i = 0; i < hallCount; i++)
        {
            output.print(_baseReadings[i]);
            if (i < hallCount - 1)
            {
                output.print(F(", "));
            }
        }
    }
    else
    {
        output.print(F("Not initialized"));
    }
    output.println();

    if (printConfig)
    {
        // Config-specific details (add more here if needed)
        // output.println(F("Configuration details here..."));
    }
    else
    {
        // Non-config details
        output.print(F("SorterSubsystem State: "));
        output.print(_state);
        output.print(F(" Detect: "));
        output.println(objectMagnet ? "True" : "False");

        // Print current readings from Hall sensors
        output.print(F("Current Hall Readings: "));
        int *currentReadings = halls.getReadings();
        if (currentReadings != nullptr)
        {
            for (int i = 0; i < hallCount; i++)
            {
                output.print(currentReadings[i]);
                if (i < hallCount - 1)
                {
                    output.print(F(", "));
                }
            }
        }
        else
        {
            output.print(F("Not initialized"));
        }
        output.println();
    }
}

#include "SorterSubsystem.h"

// Overloaded operator to print the subsystem state (printConfig = false)
Print &operator<<(Print &output, const SorterSubsystem &subsystem)
{
    subsystem.PrintInfo(output, false); // Use false by default for printConfig
    return output;
}
