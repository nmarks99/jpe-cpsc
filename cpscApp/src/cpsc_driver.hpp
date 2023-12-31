#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include <iostream>

class epicsShareClass CpscMotorAxis : public asynMotorAxis {
    public:
        CpscMotorAxis(class CpscMotorController *pC, int axisNo);

        void report(FILE *fp, int level);
        asynStatus stop(double acceleration);
        asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
        asynStatus poll(bool *moving);
        asynStatus setClosedLoop(bool closedLoop);
        asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
        
    private:
        CpscMotorController *pC_;
        int axisIndex_;
        bool once;
        double mres;
        bool fben; // feedback enabled


    friend class CpscMotorController;
};

class epicsShareClass CpscMotorController : public asynMotorController {
    public:
        CpscMotorController(const char *portName,
                            const char *CpscMotorController,
                            int numAxes, double movingPollPeriod, double idlePollPeriod);
        void report(FILE *fp, int level);
        CpscMotorAxis* getAxis(asynUser *pasynUser);
        CpscMotorAxis* getAxis(int axisNo);

    friend class CpscMotorAxis;
};
