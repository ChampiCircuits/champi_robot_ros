#ifndef INPUTS_H
#define INPUTS_H

#include "motion.h"

// Hardware input sampling: tirette cord and team selector switch.
// Call inputsInit() once from setup() before using the read functions.

void inputsInit();

// Returns the currently selected team from the physical switch.
Team inputsReadTeam();

// Returns true while the tirette cord is removed (robot start condition).
bool inputsTiretteIsActive();

#endif // INPUTS_H
