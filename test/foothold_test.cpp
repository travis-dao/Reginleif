#include <iostream>
#include "leg_controller.h"
#include "mpc.h"
#include "types.h"
using namespace std;

Array<Vec3, 4> current_foot_positions_bf = Offset::neutral_foot_pos_bf;

Vec3 nominal_target_bf = Offset::neutral_foot_pos_bf[0];
int airborne_leg = 0;
float length = Movement::k_step_length_mm;

int main() {
    nominal_target_bf.x += length / 2.0f;

    for (int i = 0; i < 4; i++) {
        printf("neutral pos (leg %d): ", i);
        current_foot_positions_bf[i].println();
    }

    current_foot_positions_bf[0].x += -length / 2.0f;
    current_foot_positions_bf[2].x += -length / 6.0f;
    current_foot_positions_bf[1].x += +length / 6.0f;
    current_foot_positions_bf[3].x += +length / 2.0f;

    for (int i = 0; i < 4; i++) {
        printf("gait starting pos (leg %d): ", i);
        current_foot_positions_bf[i].println();
    }

    FootholdMpc foothold;
    FootholdMpc::State state = {
        .current_foot_positions_bf = current_foot_positions_bf,
        .swing_leg = airborne_leg,
        .nominal_foothold_bf = nominal_target_bf
    };

    auto res = foothold.planFoothold(state);
    nominal_target_bf.println();
    res.foothold_bf.println();
    return 0;
}