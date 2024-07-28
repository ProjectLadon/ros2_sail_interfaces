// This include file contains enums used in mavlink messages and their
// corresponding ROS2 messages

#pragma once

namespace sail_interfaces
{
    /** @brief The position of a wingsail on a boat */
    enum class SAIL_POS_ID
    {
        SAIL_POS_ID_SINGLE  = 0,    /* A single sail on the boat | */
        SAIL_POS_ID_1       = 1,    /* Wingsail #1 | */
        SAIL_POS_ID_2       = 2,    /* Wingsail #2 | */
        SAIL_POS_ID_3       = 3,    /* Wingsail #3 | */
        SAIL_POS_ID_4       = 4,    /* Wingsail #4 | */
        SAIL_POS_ID_5       = 5,    /* Wingsail #5 | */
        SAIL_POS_ID_6       = 6,    /* Wingsail #6 | */
        SAIL_POS_ID_7       = 7,    /* Wingsail #7 | */
        SAIL_POS_ID_8       = 8,    /* Wingsail #8 | */
        SAIL_POS_ID_FORE    = 20,   /* The forward most sail on a boat | */
        SAIL_POS_ID_MAIN    = 21,   /* The middle sail on a boat | */
        SAIL_POS_ID_MIZZEN  = 22,   /* The aft most sail on a boat | */
        SAIL_POS_ID_NONE    = 255,  /* No wingsail defined | */
    };

    /** @brief The type of the angle of the sail command/report */
    enum class SAIL_ANGLE_TYPE 
    {
        SAIL_ANGLE_TYPE_AOA = 1,    /* The angle is an angle of attack | */
        SAIL_ANGLE_TYPE_ROT = 2,    /* The angle is a rotation angle | */
    };

    /** @brief The type of the wind data from a node */
    enum class WIND_DATA_TYPE
    {
        WIND_DATA_TYPE_RAW      = 0,    /* The wind is raw | */
        WIND_DATA_TYPE_APPARENT = 1,    /* The wind data is relative to the boat | */
        WIND_DATA_TYPE_TRUE     = 2,    /* The wind data is relative to the ground | */
        WIND_DATA_TYPE_INVALID  = 255,  /* The wind is invalid | */
    };
}
