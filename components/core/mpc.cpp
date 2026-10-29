#include "mpc.h"
#include "types.h"

FootholdMpc::FootholdMpc(const Config& cfg) {
    this->cfg = cfg;
}

FootholdMpc::Result FootholdMpc::planFoothold(const State& state) const {
    return {Vec3 {0.0f,0.0f,0.0f}, 0.0f, true};
}

float FootholdMpc::scoreCandidate(const State& state, const Vec3& candidate_foothold_body) const {
    return 0.0f;
}

bool FootholdMpc::isCandidateValid(const State& state,const Vec3& candidate_foothold_body) const {
    return true;
}