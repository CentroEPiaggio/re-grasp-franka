#ifndef QBINTERFACE_2_H
#define QBINTERFACE_2_H

#include <string.h>
#include <iostream>
#include <math.h>
#include <unistd.h>

#include "qbmove_communications.h"
#include "definitions.h"

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

enum angular_unit {RAD, DEG, TICK};
enum linear_unit {MM, M};

class qbInterface_2
{
    public:

        //------------------------------ qbInterface_2
        // Costructor of qbInterface_2 class
        qbInterface_2(const int id = 1);


        //------------------------------ qbInterface_2
        // External costructor of qbInterface_2 class
        qbInterface_2(comm_settings*, const int id = 1);


        //------------------------------ ~qbInterface_2
        // Distructor of qbInterface_2 class
        ~qbInterface_2();


        // ------------------------------
        //      Other Functions
        // ------------------------------

        //------------------------------ open
        // Open serial communication
        bool open(const char*);


        //------------------------------ close
        // Close serial communication
        void close();


        //------------------------------ activate
        // Active the cube
        bool activate();


        //------------------------------ deactivate
        // Active the cube
        bool deactivate();

        //------------------------------ getMeas
        // Get measurement of positions [1, 2, 3] in ticks
        bool getMeas(short int*);

        //------------------------------ setInputs
        // Set board inputs in ticks [1, 2]
        bool setInputs(short int*);

        //------------------------------ getID
        // Getter of variable ID
        int getID();

        //------------------------------ getMeasAndCurr
        // Get measurements and current of the motors
        bool getMeasAndCurr(short int*, short int*);


        //------------------------------ getCurrents
        // Get motor currents
        template <typename T>
        bool getCurrents(T*);

        comm_settings* cube_comm_;

    protected:
        int id_;
        int axis_dir_;

    private:

};

#include "../src/qbInterface_2.tpp"

#endif // QBINTERFACE_2_H
