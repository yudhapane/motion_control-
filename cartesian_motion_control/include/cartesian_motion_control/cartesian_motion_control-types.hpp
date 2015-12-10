#ifndef CARTESIAN_MOTION_CONTROL_TYPES_HPP
#define CARTESIAN_MOTION_CONTROL_TYPES_HPP

/**
 * In this header you may define a number of classes or structs,
 * which will be picked up by the typegen tool.
 *
 * typegen will not process headers included by this header.
 * If you want to generate type support for another header than
 * this one, add that header in the CMakeLists.txt file.
 */

#include <vector>

/**
 * Just an example struct. You may remove/modify it.
 * Note that there are restrictions. Take a look at the
 * Orocos typekit plugin manuals and the typegen documentation.
 */
struct Cartesian_motion_controlData
{
    /** Contains a sequence of doubles. */
    std::vector<double> samples;
};

#endif // CARTESIAN_MOTION_CONTROL_TYPES_HPP
