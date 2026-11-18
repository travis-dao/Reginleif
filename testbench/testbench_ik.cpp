// make -C testbench clean
// make -C testbench build
// make -C testbench run

#include "ik.h"
#include "leg_config.h"

static bool isRightLeg(const int id) {
	return id == 1 || id == 2;
}

void initLegs() {
    for (int i = 0; i < 4; i++) {
        Theta3 out_angles_deg;
        Vec3 target_foot_pos_lf = Offset::neutral_foot_pos_lf[i];
        target_foot_pos_lf.y *= isRightLeg(i) ? -1.0f : 1.0f;
        
        bool ok = IK({
            .body_coxa_x_mm = BODY_COXA_X_MM,
            .femur_length_mm = FEMUR_LENGTH_MM,
            .tibia_length_mm = TIBIA_LENGTH_MM,
            .c_min = c_min,
            .c_max = c_max
        }, target_foot_pos_lf, out_angles_deg);

        if (ok) {
            if (!isRightLeg(i)) {
                out_angles_deg = invertAngles(out_angles_deg);
            }
            printf("Leg %d | coxa=%.2f, femur=%.2f, tibia=%.2f\n", i, out_angles_deg.coxa, out_angles_deg.femur, out_angles_deg.tibia);
        }
    }
}

int main() {
    initLegs();
    return 0;
}